/*
 * iFlytek (讯飞) text-to-speech, "在线语音合成" (tts) service.
 *
 * Protocol: WebSocket, wss://<host>/v2/tts
 * Auth:     identical HMAC-SHA256 scheme to the STT service.
 *
 * The service returns base64-encoded PCM inside JSON frames. With
 * aue="raw" and auf="audio/L16;rate=16000" we get exactly the format the
 * MAX98357A wants, so no decoding step is needed.
 */

#include <stdlib.h>
#include <string.h>

#include "app_internal.h"
#include "cJSON.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_crt_bundle.h"
#include "esp_websocket_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "mbedtls/base64.h"
#include "mbedtls/md.h"

static const char *TAG = "tts";

#define BIT_DONE BIT0
#define BIT_ERR  BIT1

typedef struct {
    esp_websocket_client_handle_t ws;
    EventGroupHandle_t events;
    uint8_t *pcm;
    size_t pcm_len;
    size_t pcm_cap;
} tts_ctx_t;

/* ------------------------------------------------------------------------ */
/* URL signing (same scheme as iflytek_stt.c, kept local for clarity)        */
/* ------------------------------------------------------------------------ */

static void bytes_to_hex(const uint8_t *in, size_t in_len, char *out)
{
    static const char *d = "0123456789abcdef";
    for (size_t i = 0; i < in_len; i++) {
        out[i * 2]     = d[in[i] >> 4];
        out[i * 2 + 1] = d[in[i] & 0x0F];
    }
    out[in_len * 2] = '\0';
}

static esp_err_t build_signed_url(const char *host, const char *path,
                                  char *out, size_t out_len)
{
    const char *api_key    = app_cfg_iflytek_api_key();
    const char *api_secret = app_cfg_iflytek_api_secret();

    char date[64];
    time_t now = time(NULL);
    struct tm tm_utc;
    gmtime_r(&now, &tm_utc);
    strftime(date, sizeof(date), "%a, %d %b %Y %H:%M:%S GMT", &tm_utc);

    char origin[512];
    snprintf(origin, sizeof(origin),
             "host: %s\ndate: %s\nGET %s HTTP/1.1", host, date, path);

    uint8_t mac[32];
    const mbedtls_md_info_t *md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    mbedtls_md_hmac(md, (const uint8_t *)api_secret, strlen(api_secret),
                    (const uint8_t *)origin, strlen(origin), mac);

    /*
     * base64 of the RAW 32-byte HMAC - not of its hex rendering.
     *
     * The service verifies base64(hmac_sha256(secret, origin)). Base64-ing the
     * hex text instead produces an 88-character string that can never match,
     * and the handshake comes back 401, which reads like an invalid API key.
     */
    uint8_t b64_buf[128];
    size_t b64_len = 0;
    mbedtls_base64_encode(b64_buf, sizeof(b64_buf) - 1, &b64_len,
                          mac, sizeof(mac));
    b64_buf[b64_len] = '\0';

    char auth_raw[768];
    snprintf(auth_raw, sizeof(auth_raw),
             "api_key=\"%s\", algorithm=\"hmac-sha256\", headers=\"host date "
             "request-line\", signature=\"%s\"",
             api_key, (char *)b64_buf);

    uint8_t auth_b64[1024];
    size_t auth_len = 0;
    mbedtls_base64_encode(auth_b64, sizeof(auth_b64) - 1, &auth_len,
                          (const uint8_t *)auth_raw, strlen(auth_raw));
    auth_b64[auth_len] = '\0';

    char date_enc[128];
    size_t j = 0;
    for (size_t i = 0; date[i] && j + 4 < sizeof(date_enc); i++) {
        unsigned char c = (unsigned char)date[i];
        if (c == ' ' || c == ',' || c == ':') {
            j += snprintf(date_enc + j, sizeof(date_enc) - j, "%%%02X", c);
        } else {
            date_enc[j++] = (char)c;
        }
    }
    date_enc[j] = '\0';

    char auth_enc[1536];
    j = 0;
    for (size_t i = 0; auth_b64[i] && j + 4 < sizeof(auth_enc); i++) {
        unsigned char c = (unsigned char)auth_b64[i];
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
            auth_enc[j++] = (char)c;
        } else {
            j += snprintf(auth_enc + j, sizeof(auth_enc) - j, "%%%02X", c);
        }
    }
    auth_enc[j] = '\0';

    int n = snprintf(out, out_len,
                     "wss://%s%s?authorization=%s&date=%s&host=%s",
                     host, path, auth_enc, date_enc, host);
    if (n < 0 || (size_t)n >= out_len) {
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */

static bool append_pcm(tts_ctx_t *ctx, const uint8_t *data, size_t len)
{
    if (ctx->pcm_len + len > ctx->pcm_cap) {
        size_t want = (ctx->pcm_cap == 0) ? (len * 2) : (ctx->pcm_cap * 2);
        while (want < ctx->pcm_len + len) {
            want *= 2;
        }
        uint8_t *grown = realloc(ctx->pcm, want);
        if (grown == NULL) {
            return false;
        }
        ctx->pcm = grown;
        ctx->pcm_cap = want;
    }
    memcpy(ctx->pcm + ctx->pcm_len, data, len);
    ctx->pcm_len += len;
    return true;
}

static void ws_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    tts_ctx_t *ctx = (tts_ctx_t *)arg;
    esp_websocket_event_data_t *e = (esp_websocket_event_data_t *)data;

    switch (id) {
    case WEBSOCKET_EVENT_DATA: {
        if (e->op_code == 0x08 || e->data_len <= 0) {
            break;
        }

        /* Raw frame first: "no audio came back" is otherwise unanswerable. */
        {
            const int show = e->data_len > 220 ? 220 : (int)e->data_len;
            ESP_LOGI(TAG, "ws frame %d bytes: %.*s",
                     (int)e->data_len, show, e->data_ptr);
        }

        cJSON *root = cJSON_ParseWithLength(e->data_ptr, e->data_len);
        if (root == NULL) {
            break;
        }

        const cJSON *code = cJSON_GetObjectItem(root, "code");
        if (cJSON_IsNumber(code) && code->valueint != 0) {
            const cJSON *msg = cJSON_GetObjectItem(root, "message");
            ESP_LOGE(TAG, "server error %d: %s", code->valueint,
                     cJSON_IsString(msg) ? msg->valuestring : "?");
            xEventGroupSetBits(ctx->events, BIT_ERR);
            cJSON_Delete(root);
            break;
        }

        /* data.audio is base64 PCM; data.status == 2 marks the end */
        const cJSON *data_obj = cJSON_GetObjectItem(root, "data");
        if (cJSON_IsObject(data_obj)) {
            const cJSON *audio = cJSON_GetObjectItem(data_obj, "audio");
            if (cJSON_IsString(audio) && audio->valuestring[0] != '\0') {
                const size_t enc_len = strlen(audio->valuestring);
                const size_t cap = (enc_len * 3 / 4) + 4;
                uint8_t *dec = malloc(cap);
                if (dec != NULL) {
                    size_t dec_len = 0;
                    if (mbedtls_base64_decode(dec, cap, &dec_len,
                                              (const uint8_t *)audio->valuestring,
                                              enc_len) == 0) {
                        if (!append_pcm(ctx, dec, dec_len)) {
                            ESP_LOGE(TAG, "out of memory appending PCM");
                            xEventGroupSetBits(ctx->events, BIT_ERR);
                        }
                    }
                    free(dec);
                }
            }

            const cJSON *status = cJSON_GetObjectItem(data_obj, "status");
            if (cJSON_IsNumber(status) && status->valueint == 2) {
                xEventGroupSetBits(ctx->events, BIT_DONE);
            }
        }
        cJSON_Delete(root);
        break;
    }

    case WEBSOCKET_EVENT_ERROR:
        xEventGroupSetBits(ctx->events, BIT_ERR);
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
        xEventGroupSetBits(ctx->events, BIT_DONE);
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------------ */

esp_err_t iflytek_tts_synthesize(const char *text, void **out_pcm, size_t *out_len)
{
    if (text == NULL || *text == '\0' || out_pcm == NULL || out_len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_pcm = NULL;
    *out_len = 0;

    tts_ctx_t ctx = {
        .events = xEventGroupCreate(),
        .pcm = NULL,
        .pcm_len = 0,
        .pcm_cap = 0,
    };
    if (ctx.events == NULL) {
        return ESP_ERR_NO_MEM;
    }

    char url[2048];
    esp_err_t err = build_signed_url(app_cfg_iflytek_tts_host(), "/v2/tts",
                                     url, sizeof(url));
    if (err != ESP_OK) {
        vEventGroupDelete(ctx.events);
        return err;
    }

    /*
     * The certificate bundle is not optional. Without it mbedTLS has no CA to
     * verify wss://tts-api.xfyun.cn against, ssl_setup fails immediately and
     * the handshake times out - which reads like a network problem but is a
     * missing field here.
     */
    esp_websocket_client_config_t ws_cfg = {
        .uri = url,
        .buffer_size = 8192,
        .reconnect_timeout_ms = 0,
        .network_timeout_ms = 15000,
        .task_stack = 8192,
        .disable_auto_reconnect = true,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    ctx.ws = esp_websocket_client_init(&ws_cfg);
    if (ctx.ws == NULL) {
        vEventGroupDelete(ctx.events);
        return ESP_FAIL;
    }
    esp_websocket_register_events(ctx.ws, WEBSOCKET_EVENT_ANY, ws_event, &ctx);

    ESP_LOGI(TAG, "synthesising %u chars with voice \"%s\"",
             (unsigned)strlen(text), app_cfg_iflytek_tts_voice());

    err = esp_websocket_client_start(ctx.ws);
    if (err != ESP_OK) {
        goto cleanup;
    }

    {
        int waited = 0;
        while (!esp_websocket_client_is_connected(ctx.ws) && waited < 8000) {
            vTaskDelay(pdMS_TO_TICKS(50));
            waited += 50;
        }
        if (!esp_websocket_client_is_connected(ctx.ws)) {
            ESP_LOGE(TAG, "TTS handshake timed out");
            err = ESP_ERR_TIMEOUT;
            goto cleanup;
        }
    }

    /* --- request: one text frame is enough for a short reply --- */
    {
        cJSON *root = cJSON_CreateObject();

        cJSON *common = cJSON_CreateObject();
        cJSON_AddStringToObject(common, "app_id", app_cfg_iflytek_app_id());
        cJSON_AddItemToObject(root, "common", common);

        cJSON *business = cJSON_CreateObject();
        /* raw PCM, 16 kHz, so the amplifier can consume it directly */
        cJSON_AddStringToObject(business, "aue", "raw");
        cJSON_AddStringToObject(business, "auf", "audio/L16;rate=16000");
        cJSON_AddStringToObject(business, "vcn", app_cfg_iflytek_tts_voice());
        cJSON_AddNumberToObject(business, "speed", app_cfg_iflytek_tts_speed());
        cJSON_AddNumberToObject(business, "volume", 60);
        cJSON_AddNumberToObject(business, "pitch", 50);
        /*
         * tte is an enum STRING, not a number. Sent as 1 the service answers
         * 10163 "type must be string" and refuses the request.
         */
        cJSON_AddStringToObject(business, "tte", "UTF8"); /* UTF-8 input */
        cJSON_AddItemToObject(root, "business", business);

        cJSON *data = cJSON_CreateObject();
        cJSON_AddNumberToObject(data, "status", 2); /* 2 = whole text in one frame */
        cJSON_AddStringToObject(data, "text",
                                ""); /* replaced below with real base64 text */
        /* iFlytek wants the text base64-encoded */
        {
            const size_t tl = strlen(text);
            const size_t cap = (tl * 4 / 3) + 8;
            uint8_t *b64 = malloc(cap);
            if (b64 == NULL) {
                cJSON_Delete(root);
                err = ESP_ERR_NO_MEM;
                goto cleanup;
            }
            size_t b64_len = 0;
            mbedtls_base64_encode(b64, cap, &b64_len, (const uint8_t *)text, tl);
            b64[b64_len] = '\0';
            cJSON_ReplaceItemInObject(data, "text", cJSON_CreateString((char *)b64));
            free(b64);
        }
        cJSON_AddItemToObject(root, "data", data);

        char *json = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);
        if (json == NULL) {
            err = ESP_ERR_NO_MEM;
            goto cleanup;
        }
        int sent = esp_websocket_client_send_text(ctx.ws, json, strlen(json),
                                                  pdMS_TO_TICKS(5000));
        free(json);
        if (sent < 0) {
            err = ESP_FAIL;
            goto cleanup;
        }
    }

    /* --- collect the audio --- */
    {
        /*
         * 10 s, not 20. When the service accepts the request and then returns
         * nothing at all - which it does intermittently - every extra second
         * here is a second the caller spends waiting before the dispatcher can
         * fall back to the other backend. A working synthesis completes well
         * inside this window (first frame measured at 5.4 s).
         */
        EventBits_t bits = xEventGroupWaitBits(ctx.events, BIT_DONE | BIT_ERR,
                                               pdTRUE, pdFALSE,
                                               pdMS_TO_TICKS(10000));
        if (bits & BIT_ERR) {
            err = ESP_FAIL;
            goto cleanup;
        }
        if (!(bits & BIT_DONE)) {
            ESP_LOGW(TAG, "TTS did not signal completion, using what arrived");
        }
    }

    if (ctx.pcm == NULL || ctx.pcm_len == 0) {
        ESP_LOGE(TAG, "no audio returned");
        err = ESP_ERR_NOT_FOUND;
        goto cleanup;
    }

    ESP_LOGI(TAG, "received %u bytes of PCM (%.1f s)", (unsigned)ctx.pcm_len,
             (double)ctx.pcm_len / (16000 * 2));

    *out_pcm = ctx.pcm;
    *out_len = ctx.pcm_len;
    ctx.pcm = NULL;
    err = ESP_OK;

cleanup:
    if (ctx.ws) {
        esp_websocket_client_stop(ctx.ws);
        esp_websocket_client_destroy(ctx.ws);
    }
    free(ctx.pcm);
    vEventGroupDelete(ctx.events);
    return err;
}
