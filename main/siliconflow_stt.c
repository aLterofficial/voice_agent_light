/*
 * SiliconFlow speech-to-text (FunAudioLLM/SenseVoiceSmall).
 *
 *   POST <base>/audio/transcriptions
 *   Authorization: Bearer <key>
 *   Content-Type: multipart/form-data
 *     model = FunAudioLLM/SenseVoiceSmall
 *     file  = <the audio>
 *   ->  {"text": "..."}
 *
 * WHY THIS EXISTS ALONGSIDE iflytek_stt.c
 *
 * The iFlytek service needs AppID + APIKey + APISecret run through an
 * HMAC-SHA256 signature, carried on a WebSocket URL, with the audio base64-ed
 * into JSON frames. That is most of what makes iflytek_stt.c 430 lines long.
 * This service needs one Bearer token on an ordinary HTTPS POST, and the audio
 * goes up as raw bytes.
 *
 * The two are kept side by side and selected with CONFIG_STT_BACKEND_* so the
 * iFlytek path can be restored without touching code if a key or the free
 * quota runs out.
 *
 * WHY THE AUDIO IS WRAPPED IN A WAV HEADER
 *
 * The transcribe route is documented as taking a file object, and every
 * official example sends a container (mp3). Raw headerless PCM is therefore
 * not something to rely on. A 44-byte canonical WAV header declares the exact
 * format the microphone already produces - 16 kHz, 16-bit, mono - and costs
 * nothing, so it is always prepended rather than gambled on.
 *
 * References:
 *   https://docs.siliconflow.cn/docs/api/audio-transcriptions-post
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/time.h>

#include "app_internal.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "sf-stt";

/* Multipart boundary. Must not appear in the payload; this string cannot occur
 * in PCM audio and does not collide with the JSON form fields. */
#define BOUNDARY "----DSHFormBoundary7MA4YWxkTrZu0gW"

/* JSON reply is tiny - {"text": "..."} - but a long utterance could in theory
 * produce a long transcript, so leave generous room. */
#define RESP_CAP 4096

/* Room for the configured base URL plus "/audio/transcriptions". */
#define URL_MAX 320

/* ------------------------------------------------------------------------ */
/* URL                                                                       */
/* ------------------------------------------------------------------------ */

/*
 * Join the configured base with the route.
 *
 * Accepts "https://api.siliconflow.cn", ".../v1", and the complete endpoint
 * URL, because a mis-pasted base should produce a working request rather than
 * a 404. Same tolerance as deepseek_client.c, for the same reason.
 */
static esp_err_t build_url(char *out, size_t out_len)
{
    const char *base = app_cfg_siliconflow_base_url();

    if (base == NULL || base[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    if (strstr(base, "/audio/transcriptions") != NULL) {
        snprintf(out, out_len, "%s", base);
        return ESP_OK;
    }

    size_t n = strlen(base);
    while (n > 0 && base[n - 1] == '/') {
        n--;
    }

    /*
     * The service expects the version segment. Only add it when the base does
     * not already carry one, so both ".../v1" and a bare host work.
     */
    if (strstr(base, "/v1") != NULL) {
        snprintf(out, out_len, "%.*s/audio/transcriptions", (int)n, base);
    } else {
        snprintf(out, out_len, "%.*s/v1/audio/transcriptions", (int)n, base);
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* WAV header                                                                */
/* ------------------------------------------------------------------------ */

static void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void put_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

/*
 * Canonical 44-byte RIFF/WAVE header for 16-bit mono PCM.
 * This is enough for any decoder - there is no LIST/fact chunk to omit.
 */
static void build_wav_header(uint8_t *h, uint32_t data_len, uint32_t rate)
{
    const uint16_t channels = 1;
    const uint16_t bits     = 16;
    const uint32_t byte_rate = rate * channels * (bits / 8);
    const uint16_t block_align = (uint16_t)(channels * (bits / 8));

    memcpy(h + 0, "RIFF", 4);
    put_le32(h + 4, 36 + data_len);       /* RIFF chunk size = 36 + data */
    memcpy(h + 8, "WAVE", 4);

    memcpy(h + 12, "fmt ", 4);
    put_le32(h + 16, 16);                 /* PCM fmt chunk size */
    put_le16(h + 20, 1);                  /* audio format: PCM */
    put_le16(h + 22, channels);
    put_le32(h + 24, rate);
    put_le32(h + 28, byte_rate);
    put_le16(h + 32, block_align);
    put_le16(h + 34, bits);

    memcpy(h + 36, "data", 4);
    put_le32(h + 40, data_len);
}

/* ------------------------------------------------------------------------ */
/* Response                                                                  */
/* ------------------------------------------------------------------------ */

typedef struct {
    char buf[RESP_CAP];
    size_t len;
    bool overflow;
} resp_t;

static esp_err_t http_event(esp_http_client_event_t *e)
{
    resp_t *r = (resp_t *)e->user_data;

    switch (e->event_id) {
    case HTTP_EVENT_ON_HEADER:
        /*
         * Set the clock from the Date header.
         *
         * iFlytek's WebSocket API signs each request with the current GMT date,
         * so TTS answers 401 until the device knows what year it is. The device
         * has no RTC, and SNTP is often blocked on phone hotspots (UDP 123), so
         * the one timestamp we can always get is this one - every HTTPS
         * response carries it. It runs before the first TTS of a turn because
         * recognitions always happen first.
         */
        if (e->header_key != NULL && e->header_value != NULL &&
            strcasecmp(e->header_key, "Date") == 0) {
            struct tm tm;
            memset(&tm, 0, sizeof(tm));
            if (strptime(e->header_value, "%a, %d %b %Y %H:%M:%S", &tm) != NULL) {
                const time_t t = timegm(&tm);
                if (t > 1600000000) {
                    struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
                    settimeofday(&tv, NULL);
                    ESP_LOGI(TAG, "clock set from HTTP Date: %lld", (long long)t);
                }
            }
        }
        break;

    case HTTP_EVENT_ON_DATA:
        if (r == NULL || r->overflow) {
            break;
        }
        if (r->len + e->data_len >= RESP_CAP) {
            ESP_LOGW(TAG, "response exceeds %d bytes, truncating", RESP_CAP);
            r->overflow = true;
            size_t room = RESP_CAP - r->len - 1;
            memcpy(r->buf + r->len, e->data, room);
            r->len += room;
            r->buf[r->len] = '\0';
            break;
        }
        memcpy(r->buf + r->len, e->data, e->data_len);
        r->len += e->data_len;
        r->buf[r->len] = '\0';
        break;

    case HTTP_EVENT_ERROR:
        ESP_LOGE(TAG, "http error");
        break;

    default:
        break;
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* Public entry point                                                        */
/* ------------------------------------------------------------------------ */

esp_err_t siliconflow_stt_recognize(const void *pcm, size_t len, char **out_text)
{
    if (pcm == NULL || len == 0 || out_text == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_text = NULL;

    char url[URL_MAX];
    esp_err_t err = build_url(url, sizeof(url));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SILICONFLOW_BASE_URL is empty");
        return err;
    }

    /*
     * WAV header + payload in one allocation.
     *
     * The header is prepended in a separate buffer rather than by shifting the
     * PCM, because the source buffer is the caller's recording and must not be
     * modified - do_turn() may still need it.
     */
    const uint32_t rate = (uint32_t)app_cfg_audio_sample_rate();
    const size_t wav_len = 44 + len;

    uint8_t *wav = malloc(wav_len);
    if (wav == NULL) {
        ESP_LOGE(TAG, "no memory for the %u byte upload", (unsigned)wav_len);
        return ESP_ERR_NO_MEM;
    }
    build_wav_header(wav, (uint32_t)len, rate);
    memcpy(wav + 44, pcm, len);

    /* --- multipart body --- */
    char part_model[256];
    int part_model_len = snprintf(part_model, sizeof(part_model),
        "--" BOUNDARY "\r\n"
        "Content-Disposition: form-data; name=\"model\"\r\n"
        "\r\n"
        "%s\r\n", app_cfg_siliconflow_stt_model());
    if (part_model_len <= 0 || part_model_len >= (int)sizeof(part_model)) {
        free(wav);
        return ESP_ERR_INVALID_SIZE;
    }

    char part_file[320];
    int part_file_len = snprintf(part_file, sizeof(part_file),
        "--" BOUNDARY "\r\n"
        "Content-Disposition: form-data; name=\"file\"; filename=\"audio.wav\"\r\n"
        "Content-Type: audio/wav\r\n"
        "\r\n");
    if (part_file_len <= 0 || part_file_len >= (int)sizeof(part_file)) {
        free(wav);
        return ESP_ERR_INVALID_SIZE;
    }

    static const char tail[] = "\r\n--" BOUNDARY "--\r\n";

    const int64_t content_len = (int64_t)part_model_len + part_file_len +
                                (int64_t)wav_len + (int64_t)(sizeof(tail) - 1);

    resp_t *resp = calloc(1, sizeof(resp_t));
    if (resp == NULL) {
        free(wav);
        return ESP_ERR_NO_MEM;
    }

    esp_http_client_config_t cfg = {
        .url = url,
        .event_handler = http_event,
        .user_data = resp,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 30000,          /* upload + inference */
        .buffer_size = 4096,
        .buffer_size_tx = 4096,
        .keep_alive_enable = false,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == NULL) {
        free(wav);
        free(resp);
        return ESP_FAIL;
    }

    char auth[256];
    snprintf(auth, sizeof(auth), "Bearer %s", app_cfg_siliconflow_key());
    esp_http_client_set_header(client, "Authorization", auth);
    esp_http_client_set_header(client, "Accept", "application/json");

    /*
     * Everything that shapes the request line and its headers must be set
     * BEFORE open(): open() serialises the header block and sends it, so a
     * set_header() call afterwards silently does nothing.
     *
     * The method matters as much as the URL. open(client, 0) leaves the method
     * at its GET default, and the service answers a GET on this POST-only route
     * with a bare "Not Found" 404 - which is exactly how this bug showed up in
     * the field (HTTP 404: Not Found, every turn).
     */
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type",
                               "multipart/form-data; boundary=" BOUNDARY);

    ESP_LOGI(TAG, "POST %s", url);
    ESP_LOGI(TAG, "transcribing %u bytes (%u Hz) as %s",
             (unsigned)len, (unsigned)rate, app_cfg_siliconflow_stt_model());

    /*
     * A non-zero write length makes the client emit "Content-Length" for us
     * and is also what keeps the request a POST. The parts are still streamed
     * by hand below - nothing is buffered twice.
     */
    err = esp_http_client_open(client, (int)content_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "connect failed: %s", esp_err_to_name(err));
        goto done;
    }

    /*
     * Push exactly `len` bytes. esp_http_client_write() returns the count it
     * actually accepted, which can be short when the TLS window is full - and
     * a silently truncated body makes the request's Content-Length a lie. The
     * service then cannot parse the multipart form and answers HTTP 500 with an
     * empty body.
     */
    const uint8_t *parts[4] = { (const uint8_t *)part_model,
                                (const uint8_t *)part_file,
                                wav,
                                (const uint8_t *)tail };
    const int part_len[4] = { part_model_len, part_file_len,
                              (int)wav_len, (int)(sizeof(tail) - 1) };

    for (int k = 0; k < 4; k++) {
        const uint8_t *p = parts[k];
        int left = part_len[k];
        while (left > 0) {
            int n = esp_http_client_write(client, (const char *)p, left);
            if (n <= 0) {
                ESP_LOGE(TAG, "upload failed at part %d, %d bytes left", k, left);
                err = ESP_FAIL;
                goto done;
            }
            p += n;
            left -= n;
        }
    }

    int status = esp_http_client_fetch_headers(client);
    if (status < 0) {
        ESP_LOGE(TAG, "no response headers");
        err = ESP_FAIL;
        goto done;
    }

    int code = esp_http_client_get_status_code(client);
    /*
     * Drain the body so the ON_DATA event fires and resp->buf gets filled.
     * esp_http_client_read_response(client, buf, 0) would read nothing at all,
     * which left the error line below with an empty buffer.
     */
    {
        char sink[256];
        while (esp_http_client_read(client, sink, sizeof(sink)) > 0) {
        }
    }

    if (code != 200) {
        ESP_LOGE(TAG, "HTTP %d: %s", code, resp->buf);
        err = ESP_ERR_INVALID_RESPONSE;
        goto done;
    }

    /* --- {"text": "..."} --- */
    cJSON *root = cJSON_Parse(resp->buf);
    if (root == NULL) {
        ESP_LOGE(TAG, "reply is not JSON: %s", resp->buf);
        err = ESP_ERR_INVALID_RESPONSE;
        goto done;
    }

    const cJSON *text = cJSON_GetObjectItemCaseSensitive(root, "text");
    if (!cJSON_IsString(text) || text->valuestring == NULL) {
        ESP_LOGE(TAG, "reply has no \"text\" field: %s", resp->buf);
        cJSON_Delete(root);
        err = ESP_ERR_INVALID_RESPONSE;
        goto done;
    }

    /*
     * Leading/trailing whitespace is stripped because the text is matched
     * against command phrases by substring search; a stray leading space is
     * harmless there, but the transcript is also logged and stored in the
     * conversation history, where it is noise.
     */
    const char *s = text->valuestring;
    while (*s == ' ' || *s == '\t' || *s == '\n' || *s == '\r') {
        s++;
    }
    size_t tlen = strlen(s);
    while (tlen > 0 && (s[tlen - 1] == ' ' || s[tlen - 1] == '\t' ||
                        s[tlen - 1] == '\n' || s[tlen - 1] == '\r')) {
        tlen--;
    }

    if (tlen == 0) {
        ESP_LOGW(TAG, "empty transcript (no speech recognised)");
        cJSON_Delete(root);
        err = ESP_ERR_INVALID_SIZE;
        goto done;
    }

    char *out = malloc(tlen + 1);
    if (out == NULL) {
        cJSON_Delete(root);
        err = ESP_ERR_NO_MEM;
        goto done;
    }
    memcpy(out, s, tlen);
    out[tlen] = '\0';
    *out_text = out;
    err = ESP_OK;

    cJSON_Delete(root);

done:
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    free(wav);
    free(resp);
    return err;
}
