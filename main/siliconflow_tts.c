/*
 * SiliconFlow text-to-speech (FunAudioLLM/CosyVoice2-0.5B).
 *
 *   POST <base>/audio/speech
 *   Authorization: Bearer <key>
 *   {"model": ..., "input": ..., "voice": ..., "response_format": "pcm",
 *    "sample_rate": 16000, "speed": ..., "gain": ...}
 *   ->  raw audio bytes
 *
 * WHY response_format IS "pcm"
 *
 * The service can return mp3, opus, wav or pcm. Asking for pcm means the
 * response body is headerless little-endian 16-bit mono samples: exactly what
 * audio_output_play() expects. There is no container to parse and no decoder to
 * link, which is the whole point of choosing it.
 *
 * The sample rate is pinned to the configured audio rate (16 kHz) rather than
 * left at the 44.1 kHz default, so the bytes can go straight to the I2S
 * playback path without resampling. The service accepts 8000/16000/24000/
 * 32000/44100 for pcm, so the configured rate has to stay inside that set -
 * see the Kconfig help for AUDIO_SAMPLE_RATE.
 *
 * References:
 *   https://docs.siliconflow.cn/docs/api/audio-speech-post
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_internal.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

static const char *TAG = "sf-tts";

/* Room for the configured base URL plus "/audio/speech". */
#define URL_MAX 320

/* Refuse absurd replies early rather than growing until the heap is gone.
 * Speech runs about 32 KB/s at 16 kHz mono, so 2 MB is a full minute - far
 * beyond any reply this device produces. */
#define PCM_CAP_MAX (2 * 1024 * 1024)

/* ------------------------------------------------------------------------ */
/* URL                                                                       */
/* ------------------------------------------------------------------------ */

/* NULL = use whatever CONFIG_SILICONFLOW_* says. Set by the dispatcher when it
 * targets the LAN server, which speaks the same API with a different voice. */
static const tts_endpoint_t *s_ep = NULL;

void siliconflow_tts_set_endpoint(const tts_endpoint_t *ep)
{
    s_ep = ep;
}

static const char *cur_base(void)
{
    return (s_ep != NULL && s_ep->base_url != NULL)
               ? s_ep->base_url : app_cfg_siliconflow_base_url();
}

static const char *cur_key(void)
{
    return (s_ep != NULL && s_ep->key != NULL) ? s_ep->key : app_cfg_siliconflow_key();
}

static const char *cur_voice(void)
{
    return (s_ep != NULL && s_ep->voice != NULL) ? s_ep->voice : app_cfg_siliconflow_tts_voice();
}

static const char *cur_model(void)
{
    return (s_ep != NULL && s_ep->model != NULL) ? s_ep->model : app_cfg_siliconflow_tts_model();
}

/*
 * Local requests have to fail fast.
 *
 * With the laptop asleep the TCP connect black-holes for ~18 s at the default
 * timeout, and every turn would stall that long before the cloud fallback even
 * started. The local server's measured worst case is 0.6 s to first chunk, so
 * 5 s is generous and still quick enough to not be felt.
 */
static int cur_timeout_ms(void)
{
    return (s_ep != NULL) ? 5000 : 45000;
}

static esp_err_t build_url(char *out, size_t out_len)
{
    const char *base = cur_base();

    if (base == NULL || base[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    if (strstr(base, "/audio/speech") != NULL) {
        snprintf(out, out_len, "%s", base);
        return ESP_OK;
    }

    size_t n = strlen(base);
    while (n > 0 && base[n - 1] == '/') {
        n--;
    }

    if (strstr(base, "/v1") != NULL) {
        snprintf(out, out_len, "%.*s/audio/speech", (int)n, base);
    } else {
        snprintf(out, out_len, "%.*s/v1/audio/speech", (int)n, base);
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* Response - a growable binary buffer                                       */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint8_t *buf;
    size_t len;
    size_t cap;
    bool overflow;

    /* When sink is set the audio is handed over block by block instead of
     * being accumulated, and buf stays NULL. */
    tts_pcm_sink_t sink;
    void          *user;
    size_t         streamed;
} pcm_t;

/* Milliseconds since the current request was started, -1 before the first. */
static int64_t s_req_t0 = -1;
static bool    s_saw_first_data = false;

static int64_t since_req(void)
{
    return (s_req_t0 < 0) ? -1 : (esp_timer_get_time() - s_req_t0) / 1000;
}

static esp_err_t http_event(esp_http_client_event_t *e)
{
    pcm_t *p = (pcm_t *)e->user_data;

    /*
     * Phase timing. A 3 s gap between "request sent" and "first byte" was
     * observed with a server that answers in 0.6 s, and without these markers
     * there is no way to tell a slow connect from a slow first byte.
     */
    switch (e->event_id) {
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG, "http: connected at %lld ms", (long long)since_req());
        break;
    case HTTP_EVENT_ON_HEADER:
        break;
    case HTTP_EVENT_ON_DATA:
        if (p == NULL || p->overflow || e->data_len <= 0) {
            break;
        }
        if (!s_saw_first_data) {
            s_saw_first_data = true;
            ESP_LOGI(TAG, "http: first body byte at %lld ms",
                     (long long)since_req());
        }
        if (p->sink != NULL) {
            if (p->sink(e->data, (size_t)e->data_len, p->user) != ESP_OK) {
                p->overflow = true;
                break;
            }
            p->streamed += (size_t)e->data_len;
            break;
        }
        if (p->len + (size_t)e->data_len > PCM_CAP_MAX) {
            ESP_LOGE(TAG, "reply exceeds the %d byte cap", PCM_CAP_MAX);
            p->overflow = true;
            break;
        }
        if (p->len + (size_t)e->data_len > p->cap) {
            /* Double, so N chunks cost O(log N) reallocations. */
            size_t want = p->cap ? p->cap * 2 : 16384;
            while (want < p->len + (size_t)e->data_len) {
                want *= 2;
            }
            uint8_t *nb = realloc(p->buf, want);
            if (nb == NULL) {
                ESP_LOGE(TAG, "no memory to grow the audio buffer to %u",
                         (unsigned)want);
                p->overflow = true;
                break;
            }
            p->buf = nb;
            p->cap = want;
        }
        memcpy(p->buf + p->len, e->data, e->data_len);
        p->len += e->data_len;
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

static esp_err_t tts_request(const char *text, void **out_pcm, size_t *out_len,
                             tts_pcm_sink_t sink, void *user, bool stream)
{
    if (text == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!stream && (out_pcm == NULL || out_len == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (out_pcm != NULL) {
        *out_pcm = NULL;
    }
    if (out_len != NULL) {
        *out_len = 0;
    }

    if (text[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    char url[URL_MAX];
    esp_err_t err = build_url(url, sizeof(url));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SILICONFLOW_BASE_URL is empty");
        return err;
    }

    /* --- request body --- */
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }
    /* AddStringToObject escapes the text, so quotes, newlines and any UTF-8
     * the model produced are safe - the "no spaces" note in the docs is an
     * artefact of their curl examples, not a JSON-level restriction. */
    cJSON_AddStringToObject(root, "model", cur_model());
    cJSON_AddStringToObject(root, "input", text);
    cJSON_AddStringToObject(root, "voice", cur_voice());
    cJSON_AddStringToObject(root, "response_format", "pcm");
    cJSON_AddNumberToObject(root, "sample_rate", app_cfg_audio_out_sample_rate());
    cJSON_AddRawToObject(root, "speed", app_cfg_siliconflow_tts_speed());
    cJSON_AddRawToObject(root, "gain", app_cfg_siliconflow_tts_gain());
    if (stream) {
        cJSON_AddBoolToObject(root, "stream", true);
    }

    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (body == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "POST %s", url);
    ESP_LOGI(TAG, "body(%u): %s", (unsigned)strlen(body), body);

    pcm_t pcm = { 0 };
    pcm.sink = sink;
    pcm.user = user;
    esp_http_client_config_t cfg = {
        .url = url,
        .event_handler = http_event,
        .user_data = &pcm,
        .crt_bundle_attach = esp_crt_bundle_attach,
        /* CosyVoice2 is autoregressive: a 6 s clip has been measured at 18 s
         * of synthesis, so 30 s is too close to the edge to be safe. */
        .timeout_ms = cur_timeout_ms(),
        /* 8192 bytes is 170 ms of 24 kHz stereo - half the DMA depth, so a
         * read always has cushion behind it. */
        .buffer_size = 8192,
        .buffer_size_tx = 2048,
        .keep_alive_enable = false,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == NULL) {
        free(body);
        return ESP_FAIL;
    }

    /* A LAN server usually ignores auth; sending an empty Bearer is pointless. */
    if (cur_key() != NULL && cur_key()[0] != '\0') {
        char auth[256];
        snprintf(auth, sizeof(auth), "Bearer %s", cur_key());
        esp_http_client_set_header(client, "Authorization", auth);
    }
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept", "audio/pcm");
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_post_field(client, body, (int)strlen(body));

    ESP_LOGI(TAG, "synthesising %u bytes of UTF-8 as %s (%s) -> %d Hz",
             (unsigned)strlen(text), cur_model(), cur_voice(),
             app_cfg_audio_out_sample_rate());

    s_req_t0 = esp_timer_get_time();
    s_saw_first_data = false;

    err = esp_http_client_perform(client);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "request failed: %s", esp_err_to_name(err));
        goto done;
    }

    const int code = esp_http_client_get_status_code(client);
    if (code != 200) {
        /*
         * An error body is JSON, not audio, and it was collected as bytes.
         * Print it as text - it is NUL-terminated by hand below so a shared
         * log line can show the service's own explanation.
         */
        if (pcm.buf != NULL) {
            uint8_t *nb = realloc(pcm.buf, pcm.len + 1);
            if (nb != NULL) {
                pcm.buf = nb;
                pcm.buf[pcm.len] = '\0';
                ESP_LOGE(TAG, "HTTP %d: %s", code, (const char *)pcm.buf);
            } else {
                ESP_LOGE(TAG, "HTTP %d", code);
            }
        } else {
            ESP_LOGE(TAG, "HTTP %d (empty body)", code);
        }
        err = ESP_ERR_INVALID_RESPONSE;
        goto done;
    }

    if (stream) {
        if (pcm.streamed == 0) {
            ESP_LOGE(TAG, "200 but no audio arrived");
            err = ESP_ERR_INVALID_SIZE;
            goto done;
        }
        ESP_LOGI(TAG, "streamed %u bytes of PCM (%.1f s at %d Hz)",
                 (unsigned)pcm.streamed,
                 (double)pcm.streamed / (2.0 * app_cfg_audio_out_sample_rate()),
                 app_cfg_audio_out_sample_rate());
        err = ESP_OK;
        goto done;
    }

    if (pcm.buf == NULL || pcm.len == 0) {
        ESP_LOGE(TAG, "200 with an empty body");
        err = ESP_ERR_INVALID_SIZE;
        goto done;
    }

    ESP_LOGI(TAG, "got %u bytes of PCM (%.1f s at %d Hz)", (unsigned)pcm.len,
             (double)pcm.len / (2.0 * app_cfg_audio_out_sample_rate()),
             app_cfg_audio_out_sample_rate());

    *out_pcm = pcm.buf;
    *out_len = pcm.len;
    pcm.buf = NULL;   /* ownership handed to the caller */
    err = ESP_OK;

done:
    esp_http_client_cleanup(client);
    free(pcm.buf);
    free(body);
    return err;
}

/* ------------------------------------------------------------------------ */
/* Public wrappers                                                           */
/* ------------------------------------------------------------------------ */

esp_err_t siliconflow_tts_synthesize(const char *text, void **out_pcm,
                                     size_t *out_len)
{
    return tts_request(text, out_pcm, out_len, NULL, NULL, false);
}

esp_err_t siliconflow_tts_stream(const char *text, tts_pcm_sink_t sink, void *user)
{
    if (sink == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return tts_request(text, NULL, NULL, sink, user, true);
}
