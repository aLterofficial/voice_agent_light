/*
 * DeepSeek chat client (OpenAI-compatible Chat Completions API).
 *
 *   POST <base-url>/chat/completions
 *   Authorization: Bearer <key>
 *
 * The base URL is configurable because third-party relays (中转站) hand out
 * keys that only work against their own host: a relay key sent to
 * api.deepseek.com is rejected with 401. See CONFIG_DEEPSEEK_BASE_URL.
 *
 * Note on the model name: the official service only accepts "deepseek-flash"
 * (DeepSeek-V4.1-Flash) and "deepseek-v4-pro". The string is passed through
 * unchanged, so a relay that documents different names still works.
 *
 * The reply is synthesised into speech, so thinking mode is disabled by
 * default - reasoning tokens would add latency and cost, and would be spoken
 * aloud if they leaked into the content field. Some relays reject the
 * unknown "thinking"/"reasoning_effort" fields with HTTP 400, which is what
 * CONFIG_DEEPSEEK_SEND_THINKING_PARAMS is for.
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
#include "sdkconfig.h"

static const char *TAG = "deepseek";

#define RESP_CAP 8192

/* Room for the configured base URL plus "/chat/completions". */
#define URL_MAX 256

/*
 * Build the full request URL from the configured base.
 *
 * The base may or may not carry a path ("https://api.deepseek.com" and
 * "https://router.example.com/api/v1" are both valid), and the user may also
 * paste the complete URL by mistake. All three cases are accepted here so a
 * mis-pasted value produces a working request instead of a 404.
 */
static esp_err_t build_url(char *out, size_t out_len)
{
    const char *base = app_cfg_deepseek_base_url();

    if (base == NULL || base[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    /* Already a full endpoint? Use as-is. */
    if (strstr(base, "/chat/completions") != NULL) {
        snprintf(out, out_len, "%s", base);
        return ESP_OK;
    }

    /* Strip any trailing slashes so the join does not produce "//". */
    size_t n = strlen(base);
    while (n > 0 && base[n - 1] == '/') {
        n--;
    }
    if (n >= out_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    snprintf(out, out_len, "%.*s/chat/completions", (int)n, base);
    return ESP_OK;
}

typedef struct {
    char buf[RESP_CAP];
    size_t len;
    bool overflow;
} resp_t;

static esp_err_t http_event(esp_http_client_event_t *e)
{
    resp_t *r = (resp_t *)e->user_data;

    switch (e->event_id) {
    case HTTP_EVENT_ON_DATA:
        if (r == NULL) {
            break;
        }
        if (r->overflow) {
            break;
        }
        if (r->len + e->data_len >= RESP_CAP) {
            ESP_LOGW(TAG, "response exceeds %d bytes, truncating", RESP_CAP);
            r->overflow = true;
            /* keep what fits so the caller still gets a usable answer */
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

esp_err_t deepseek_client_init(void)
{
    char url[URL_MAX];
    if (build_url(url, sizeof(url)) != ESP_OK) {
        ESP_LOGE(TAG, "DEEPSEEK_BASE_URL is empty");
        return ESP_ERR_INVALID_ARG;
    }

    if (strstr(app_cfg_deepseek_key(), "REPLACE_ME") != NULL) {
        ESP_LOGW(TAG, "DeepSeek API key is still the placeholder value");
    } else {
        ESP_LOGI(TAG, "DeepSeek ready: %s (model %s)", url, app_cfg_deepseek_model());
        if (strncmp(app_cfg_deepseek_key(), "sk-", 3) != 0) {
            /* Not an error - relay keys often look nothing like this. It is
             * worth one line because a relay key paired with the official
             * URL is the single most common setup mistake. */
            ESP_LOGW(TAG, "the key does not start with \"sk-\" - that is fine "
                          "for a relay (中转站), but make sure DEEPSEEK_BASE_URL "
                          "points at the relay and not at api.deepseek.com");
        }
    }
    return ESP_OK;
}

esp_err_t deepseek_client_chat(const chat_msg_t *history, size_t history_len,
                               char **out_reply)
{
    if (history == NULL || out_reply == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_reply = NULL;

    /* --- build the request body --- */
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "model", app_cfg_deepseek_model());
    cJSON_AddNumberToObject(root, "max_tokens", app_cfg_deepseek_max_tokens());
    cJSON_AddBoolToObject(root, "stream", false);

#if CONFIG_DEEPSEEK_SEND_THINKING_PARAMS
    /* Non-thinking mode: the answer is spoken, so we want it fast and
     * without a reasoning preamble. Omitted entirely when the option is off,
     * because some relays reject unknown fields with HTTP 400. */
    {
        cJSON *thinking = cJSON_CreateObject();
        cJSON_AddStringToObject(thinking, "type", "disabled");
        cJSON_AddItemToObject(root, "thinking", thinking);
        cJSON_AddStringToObject(root, "reasoning_effort", "none");
    }
#endif

    cJSON *messages = cJSON_CreateArray();
    {
        cJSON *sys = cJSON_CreateObject();
        cJSON_AddStringToObject(sys, "role", "system");
        cJSON_AddStringToObject(sys, "content", app_cfg_deepseek_system_prompt());
        cJSON_AddItemToArray(messages, sys);

        for (size_t i = 0; i < history_len; i++) {
            cJSON *m = cJSON_CreateObject();
            cJSON_AddStringToObject(m, "role", history[i].role);
            cJSON_AddStringToObject(m, "content", history[i].content);
            cJSON_AddItemToArray(messages, m);
        }
    }
    cJSON_AddItemToObject(root, "messages", messages);

    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (body == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGD(TAG, "request: %s", body);

    /* --- perform the request --- */
    resp_t resp = {0};

    char auth[256];
    snprintf(auth, sizeof(auth), "Bearer %s", app_cfg_deepseek_key());

    char url[URL_MAX];
    if (build_url(url, sizeof(url)) != ESP_OK) {
        free(body);
        return ESP_ERR_INVALID_ARG;
    }

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .event_handler = http_event,
        .user_data = &resp,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 30000,
        .buffer_size = 2048,
        .buffer_size_tx = 4096,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == NULL) {
        free(body);
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Authorization", auth);
    esp_http_client_set_post_field(client, body, strlen(body));

    ESP_LOGI(TAG, "asking DeepSeek ...");
    esp_err_t err = esp_http_client_perform(client);

    const int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    free(body);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "request failed: %s", esp_err_to_name(err));
        return err;
    }
    if (status != 200) {
        ESP_LOGE(TAG, "HTTP %d from %s", status, url);
        ESP_LOGE(TAG, "  body: %s", resp.buf);
        if (status == 401) {
            ESP_LOGE(TAG, "  401 = the key was rejected. For a relay (中转站) "
                          "key, check that DEEPSEEK_BASE_URL points at the "
                          "relay rather than api.deepseek.com");
        } else if (status == 404) {
            ESP_LOGE(TAG, "  404 = the URL is wrong. Check DEEPSEEK_BASE_URL; "
                          "the client appends /chat/completions");
        } else if (status == 400) {
            ESP_LOGE(TAG, "  400 = the request was rejected. If the body "
                          "mentions an unknown parameter, try turning off "
                          "DEEPSEEK_SEND_THINKING_PARAMS");
        }
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* --- parse choices[0].message.content --- */
    cJSON *parsed = cJSON_Parse(resp.buf);
    if (parsed == NULL) {
        ESP_LOGE(TAG, "malformed JSON: %s", resp.buf);
        return ESP_ERR_INVALID_RESPONSE;
    }

    err = ESP_ERR_NOT_FOUND;
    const cJSON *choices = cJSON_GetObjectItem(parsed, "choices");
    if (cJSON_IsArray(choices)) {
        const cJSON *choice0 = cJSON_GetArrayItem(choices, 0);
        if (cJSON_IsObject(choice0)) {
            const cJSON *message = cJSON_GetObjectItem(choice0, "message");
            if (cJSON_IsObject(message)) {
                const cJSON *content = cJSON_GetObjectItem(message, "content");
                if (cJSON_IsString(content) && content->valuestring[0] != '\0') {
                    *out_reply = strdup(content->valuestring);
                    err = (*out_reply != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
                }
            }
        }

        const cJSON *usage = cJSON_GetObjectItem(parsed, "usage");
        if (cJSON_IsObject(usage)) {
            const cJSON *pt = cJSON_GetObjectItem(usage, "prompt_tokens");
            const cJSON *ct = cJSON_GetObjectItem(usage, "completion_tokens");
            ESP_LOGI(TAG, "tokens: %d in / %d out",
                     cJSON_IsNumber(pt) ? pt->valueint : 0,
                     cJSON_IsNumber(ct) ? ct->valueint : 0);
        }
    }

    cJSON_Delete(parsed);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "reply: %s", *out_reply);
    } else {
        ESP_LOGE(TAG, "no content in response");
    }
    return err;
}
