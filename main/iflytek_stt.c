/*
 * iFlytek (讯飞) speech-to-text, "语音听写" (iat) service.
 *
 * Protocol: WebSocket, wss://<host>/v2/iat
 * Auth:     the URL carries an HMAC-SHA256 signature built from
 *           AppID + APIKey + APISecret, per the iFlytek WebSocket spec.
 *
 * Audio must be 16 kHz / 16-bit / mono PCM. Frames are base64-encoded into
 * JSON, 40 ms per frame, with a 40 ms interval between sends.
 *
 * References:
 *   https://www.xfyun.cn/doc/asr/voicedictation/API.html
 */

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "app_internal.h"
#include "cJSON.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_websocket_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "mbedtls/base64.h"
#include "mbedtls/md.h"

static const char *TAG = "stt";

/* 40 ms of 16 kHz 16-bit mono */
#define FRAME_BYTES (16000 * 2 * 40 / 1000)

#define BIT_DONE  BIT0
#define BIT_ERR   BIT1

typedef struct {
    esp_websocket_client_handle_t ws;
    EventGroupHandle_t events;
    SemaphoreHandle_t lock;
    char *text;      /* accumulated recognition result */
    size_t text_len;
    bool finished;
} stt_ctx_t;

/* ------------------------------------------------------------------------ */
/* URL signing                                                               */
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

/*
 * Build the signed wss:// URL for the iFlytek iat service.
 * `out` must have room for at least 1024 bytes.
 */
static esp_err_t build_signed_url(const char *host, const char *path,
                                  char *out, size_t out_len)
{
    const char *app_id     = app_cfg_iflytek_app_id();
    const char *api_key    = app_cfg_iflytek_api_key();
    const char *api_secret = app_cfg_iflytek_api_secret();

    /* RFC1123 date, GMT */
    char date[64];
    time_t now = time(NULL);
    struct tm tm_utc;
    gmtime_r(&now, &tm_utc);
    strftime(date, sizeof(date), "%a, %d %b %Y %H:%M:%S GMT", &tm_utc);

    /* signature origin = "host: ...\ndate: ...\nGET <path> HTTP/1.1" */
    char origin[512];
    snprintf(origin, sizeof(origin),
             "host: %s\ndate: %s\nGET %s HTTP/1.1", host, date, path);

    uint8_t mac[32];
    const mbedtls_md_info_t *md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    mbedtls_md_hmac(md, (const uint8_t *)api_secret, strlen(api_secret),
                    (const uint8_t *)origin, strlen(origin), mac);
    char sign_hex[65];
    bytes_to_hex(mac, sizeof(mac), sign_hex);

    /* base64 of the signature hex string */
    uint8_t b64_buf[128];
    size_t b64_len = 0;
    mbedtls_base64_encode(b64_buf, sizeof(b64_buf) - 1, &b64_len,
                          (const uint8_t *)sign_hex, strlen(sign_hex));
    b64_buf[b64_len] = '\0';

    /* authorization is itself base64 of the assembled fields */
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

    /* url-encode date and authorization */
    char date_enc[128];
    char auth_enc[1536];
    /* The only characters needing escaping in RFC1123 dates are spaces,
     * commas and colons. */
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

    j = 0;
    for (size_t i = 0; auth_b64[i] && j + 4 < sizeof(auth_enc); i++) {
        unsigned char c = auth_b64[i];
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
        ESP_LOGE(TAG, "signed URL too long (%d bytes)", n);
        return ESP_ERR_INVALID_SIZE;
    }
    ESP_LOGD(TAG, "url signed ok (%d bytes)", n);
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* WebSocket handling                                                        */
/* ------------------------------------------------------------------------ */

static void append_text(stt_ctx_t *ctx, const char *s)
{
    if (s == NULL || *s == '\0') {
        return;
    }
    size_t add = strlen(s);
    char *grown = realloc(ctx->text, ctx->text_len + add + 1);
    if (grown == NULL) {
        return;
    }
    memcpy(grown + ctx->text_len, s, add);
    ctx->text_len += add;
    grown[ctx->text_len] = '\0';
    ctx->text = grown;
}

static void ws_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    stt_ctx_t *ctx = (stt_ctx_t *)arg;
    esp_websocket_event_data_t *e = (esp_websocket_event_data_t *)data;

    switch (id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGD(TAG, "ws connected");
        break;

    case WEBSOCKET_EVENT_DATA: {
        if (e->op_code == 0x08) { /* close */
            ESP_LOGD(TAG, "ws closed by server");
            break;
        }
        if (e->data_len <= 0) {
            break;
        }

        /*
         * The payload may be split across frames. Reassembling by offset is
         * what esp-websocket-client expects callers to do.
         */
        cJSON *root = cJSON_ParseWithLength(e->data_ptr, e->data_len);
        if (root == NULL) {
            /* partial frame - ignore; the final frame carries the whole body
             * for the small messages iFlytek sends */
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

        /* data.result.ws[].cw[].w */
        const cJSON *data_obj = cJSON_GetObjectItem(root, "data");
        if (cJSON_IsObject(data_obj)) {
            const cJSON *result = cJSON_GetObjectItem(data_obj, "result");
            if (cJSON_IsObject(result)) {
                const cJSON *ws_arr = cJSON_GetObjectItem(result, "ws");
                if (cJSON_IsArray(ws_arr)) {
                    const cJSON *ws_item = NULL;
                    cJSON_ArrayForEach(ws_item, ws_arr) {
                        const cJSON *cw_arr = cJSON_GetObjectItem(ws_item, "cw");
                        const cJSON *cw = NULL;
                        cJSON_ArrayForEach(cw, cw_arr) {
                            const cJSON *w = cJSON_GetObjectItem(cw, "w");
                            if (cJSON_IsString(w)) {
                                append_text(ctx, w->valuestring);
                            }
                        }
                    }
                }
            }
            const cJSON *status = cJSON_GetObjectItem(data_obj, "status");
            if (cJSON_IsNumber(status) && status->valueint == 2) {
                ctx->finished = true;
                xEventGroupSetBits(ctx->events, BIT_DONE);
            }
        }
        cJSON_Delete(root);
        break;
    }

    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGE(TAG, "ws error");
        xEventGroupSetBits(ctx->events, BIT_ERR);
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "ws disconnected");
        xEventGroupSetBits(ctx->events, BIT_DONE);
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------------ */

static esp_err_t send_frame(stt_ctx_t *ctx, const void *pcm, size_t len, int status)
{
    size_t b64_cap = (len * 4 / 3) + 8;
    uint8_t *b64 = malloc(b64_cap);
    if (b64 == NULL) {
        return ESP_ERR_NO_MEM;
    }
    size_t b64_len = 0;
    mbedtls_base64_encode(b64, b64_cap, &b64_len, (const uint8_t *)pcm, len);
    b64[b64_len] = '\0';

    cJSON *root = cJSON_CreateObject();
    if (status == 0) {
        /* first frame carries the business parameters */
        cJSON_AddStringToObject(root, "common",
                                "{\"app_id\":\"" CONFIG_IFLYTEK_APP_ID "\"}");
        cJSON_AddStringToObject(
            root, "business",
            "{\"language\":\"zh_cn\",\"domain\":\"iat\",\"accent\":\"mandarin\","
            "\"vad_eos\":3000,\"dwa\":\"wpgs\"}");
    }
    cJSON_AddNumberToObject(root, "status", status); /* 0 first, 1 middle, 2 last */

    cJSON *data = cJSON_CreateObject();
    cJSON_AddStringToObject(data, "format", "audio/L16;rate=16000");
    cJSON_AddStringToObject(data, "encoding", "raw");
    cJSON_AddStringToObject(data, "audio", (char *)b64);
    cJSON_AddItemToObject(root, "data", data);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    free(b64);

    if (json == NULL) {
        return ESP_ERR_NO_MEM;
    }

    int sent = esp_websocket_client_send_text(ctx->ws, json, strlen(json),
                                              pdMS_TO_TICKS(5000));
    free(json);

    return (sent < 0) ? ESP_FAIL : ESP_OK;
}

/* ------------------------------------------------------------------------ */

esp_err_t iflytek_stt_recognize(const void *pcm, size_t len, char **out_text)
{
    if (pcm == NULL || len == 0 || out_text == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_text = NULL;

    stt_ctx_t ctx = {
        .events = xEventGroupCreate(),
        .text = NULL,
        .text_len = 0,
        .finished = false,
    };
    if (ctx.events == NULL) {
        return ESP_ERR_NO_MEM;
    }

    char url[2048];
    esp_err_t err = build_signed_url(app_cfg_iflytek_stt_host(), "/v2/iat",
                                     url, sizeof(url));
    if (err != ESP_OK) {
        vEventGroupDelete(ctx.events);
        return err;
    }

    esp_websocket_client_config_t ws_cfg = {
        .uri = url,
        .buffer_size = 8192,
        .reconnect_timeout_ms = 0,
        .network_timeout_ms = 15000,
        .task_stack = 8192,
        .disable_auto_reconnect = true,
    };

    ctx.ws = esp_websocket_client_init(&ws_cfg);
    if (ctx.ws == NULL) {
        vEventGroupDelete(ctx.events);
        return ESP_FAIL;
    }
    esp_websocket_register_events(ctx.ws, WEBSOCKET_EVENT_ANY, ws_event, &ctx);

    ESP_LOGI(TAG, "opening STT session (%u bytes of audio)", (unsigned)len);
    err = esp_websocket_client_start(ctx.ws);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ws start failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    /* Wait for the handshake (the CONNECTED event does not surface through
     * the event group, so poll the client state). */
    {
        int waited = 0;
        while (!esp_websocket_client_is_connected(ctx.ws) && waited < 8000) {
            vTaskDelay(pdMS_TO_TICKS(50));
            waited += 50;
        }
        if (!esp_websocket_client_is_connected(ctx.ws)) {
            ESP_LOGE(TAG, "STT handshake timed out");
            err = ESP_ERR_TIMEOUT;
            goto cleanup;
        }
    }

    /* Stream the audio in 40 ms frames. */
    {
        const uint8_t *p = (const uint8_t *)pcm;
        size_t remaining = len;
        size_t offset = 0;
        int status = 0;

        while (remaining > 0) {
            size_t chunk = remaining > FRAME_BYTES ? FRAME_BYTES : remaining;
            remaining -= chunk;
            offset += chunk;
            if (remaining == 0) {
                status = 2; /* last frame */
            } else {
                status = (offset == chunk) ? 0 : 1;
            }

            if (send_frame(&ctx, p, chunk, status) != ESP_OK) {
                ESP_LOGE(TAG, "frame send failed at offset %u", (unsigned)offset);
                err = ESP_FAIL;
                goto cleanup;
            }
            p += chunk;

            /* iFlytek asks for roughly real-time pacing */
            vTaskDelay(pdMS_TO_TICKS(40));
        }
    }

    /* Wait for the final result. */
    {
        EventBits_t bits = xEventGroupWaitBits(ctx.events, BIT_DONE | BIT_ERR,
                                               pdTRUE, pdFALSE,
                                               pdMS_TO_TICKS(15000));
        if (bits & BIT_ERR) {
            err = ESP_FAIL;
            goto cleanup;
        }
        if (!(bits & BIT_DONE)) {
            ESP_LOGW(TAG, "no final result within 15 s, using partial text");
        }
    }

    if (ctx.text == NULL || ctx.text_len == 0) {
        ESP_LOGW(TAG, "STT produced no text (silence or unrecognised speech)");
        err = ESP_ERR_NOT_FOUND;
        goto cleanup;
    }

    ESP_LOGI(TAG, "recognised: \"%s\"", ctx.text);
    *out_text = ctx.text;
    ctx.text = NULL;
    err = ESP_OK;

cleanup:
    if (ctx.ws) {
        esp_websocket_client_stop(ctx.ws);
        esp_websocket_client_destroy(ctx.ws);
    }
    free(ctx.text);
    vEventGroupDelete(ctx.events);
    return err;
}
