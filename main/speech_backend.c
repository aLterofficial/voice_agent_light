/*
 * Speech backend selection.
 *
 * Both STT and TTS providers stay compiled. This file is the single place that
 * knows which one is active, so the conversational turn in main.c contains no
 * preprocessor branches and a third provider would only need a new arm here.
 *
 * The point of keeping both rather than replacing iFlytek outright is
 * practical: if a key expires, a free quota runs out, or one service has an
 * outage, restoring the other is a config change and a reflash rather than an
 * edit to the request code.
 *
 * See Kconfig.projbuild -> "Speech backends" for the two choices.
 */

#include <string.h>

#include "app_internal.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "speech";

/*
 * The local Qwen3-TTS server. Same protocol as SiliconFlow, different URL and
 * voice; it is tried first so the common case never leaves the LAN.
 */
#if CONFIG_TTS_LOCAL_ENABLE
static const tts_endpoint_t k_local_ep = {
    .base_url = CONFIG_TTS_LOCAL_BASE_URL,
    .key      = "",
    .voice    = CONFIG_TTS_LOCAL_VOICE,
    .model    = NULL,
};
#endif

esp_err_t stt_recognize(const void *pcm, size_t len, char **out_text)
{
#if CONFIG_STT_BACKEND_SILICONFLOW
    return siliconflow_stt_recognize(pcm, len, out_text);
#elif CONFIG_STT_BACKEND_IFLYTEK
    return iflytek_stt_recognize(pcm, len, out_text);
#else
#error "No STT backend selected - set CONFIG_STT_BACKEND_* in menuconfig."
#endif
}

esp_err_t tts_synthesize(const char *text, void **out_pcm, size_t *out_len)
{
#if CONFIG_TTS_LOCAL_ENABLE
    {
        siliconflow_tts_set_endpoint(&k_local_ep);
        esp_err_t err = siliconflow_tts_synthesize(text, out_pcm, out_len);
        siliconflow_tts_set_endpoint(NULL);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "local tts failed (%s) - falling back to the cloud",
                 esp_err_to_name(err));
    }
#endif
#if CONFIG_TTS_BACKEND_SILICONFLOW
    return siliconflow_tts_synthesize(text, out_pcm, out_len);
#elif CONFIG_TTS_BACKEND_IFLYTEK
    return iflytek_tts_synthesize(text, out_pcm, out_len);
#else
#error "No TTS backend selected - set CONFIG_TTS_BACKEND_* in menuconfig."
#endif
}

/*
 * Streaming counterpart, with retry and cross-backend failover.
 *
 * Both services are unreliable in different ways: SiliconFlow's CosyVoice2
 * swings between 2.5 s and 14 s for the first byte and occasionally answers
 * 500, while iFlytek periodically returns no audio at all (twenty seconds of
 * silence, which looks like a dead socket). Neither is bad enough to drop, and
 * neither is good enough to rely on alone, so each synthesis gets a couple of
 * tries on the preferred backend and then a try on the other one.
 *
 * The one rule that keeps this safe: failover is only allowed while nothing has
 * been played. Once audio has reached the amplifier, retrying would repeat the
 * first words of the sentence, which sounds worse than the failure.
 */

typedef struct {
    tts_pcm_sink_t inner;
    void          *user;
    size_t         bytes;
} counted_sink_t;

static esp_err_t counted_sink(const void *pcm, size_t len, void *user)
{
    counted_sink_t *c = (counted_sink_t *)user;
    c->bytes += len;
    return c->inner(pcm, len, c->user);
}

/* One attempt on one backend. Returns ESP_OK only if audio was produced. */
static esp_err_t tts_attempt(bool siliconflow, const char *text,
                             tts_pcm_sink_t sink, void *user)
{
    if (siliconflow) {
        return siliconflow_tts_stream(text, sink, user);
    }

    /* iFlytek hands over a finished buffer, so it is pushed through in one go. */
    void *pcm = NULL;
    size_t len = 0;
    esp_err_t err = iflytek_tts_synthesize(text, &pcm, &len);
    if (err != ESP_OK) {
        return err;
    }
    err = sink(pcm, len, user);
    free(pcm);
    return err;
}

/*
 * The local Qwen3-TTS server speaks the same protocol as SiliconFlow but lives
 * on the LAN, so it is an accelerator rather than a dependency: it is tried
 * first and the configured cloud backend still runs when the laptop is asleep
 * or off the network.
 */
typedef struct {
    const char           *name;
    bool                  siliconflow;
    const tts_endpoint_t *ep;     /* NULL = the configured cloud backend */
} tts_slot_t;

/* One attempt. `played` reports bytes that already reached the amplifier. */
static esp_err_t tts_try(const tts_slot_t *slot, const char *text,
                         tts_pcm_sink_t sink, void *user, size_t *played)
{
    counted_sink_t cs = { .inner = sink, .user = user, .bytes = 0 };

    siliconflow_tts_set_endpoint(slot->ep);
    const int64_t t0 = esp_timer_get_time();
    esp_err_t err = tts_attempt(slot->siliconflow, text, counted_sink, &cs);
    siliconflow_tts_set_endpoint(NULL);

    const int64_t ms = (esp_timer_get_time() - t0) / 1000;
    *played = cs.bytes;

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "%s ok in %lld ms", slot->name, (long long)ms);
    } else if (cs.bytes > 0) {
        ESP_LOGW(TAG, "%s failed after %u bytes of audio - not retrying "
                      "(replaying would repeat the first words)",
                 slot->name, (unsigned)cs.bytes);
    } else {
        ESP_LOGW(TAG, "%s failed after %lld ms: %s",
                 slot->name, (long long)ms, esp_err_to_name(err));
    }
    return err;
}

esp_err_t tts_stream(const char *text, tts_pcm_sink_t sink, void *user)
{
    if (text == NULL || sink == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const tts_slot_t slots[] = {
#if CONFIG_TTS_LOCAL_ENABLE
        { "local", true, &k_local_ep },
#endif
#if CONFIG_TTS_BACKEND_SILICONFLOW
        { "siliconflow", true, NULL },
        { "iflytek", false, NULL },
#else
        { "iflytek", false, NULL },
        { "siliconflow", true, NULL },
#endif
    };
    const int n = (int)(sizeof(slots) / sizeof(slots[0]));

    esp_err_t last = ESP_FAIL;

    /*
     * Pass 1: exactly one try per slot. A backend that is down costs a full
     * timeout per attempt, and 2 x 21 s was measured before the healthy one was
     * even asked, so one shot each keeps the worst case to a single timeout
     * before the fallback runs.
     */
    for (int i = 0; i < n; i++) {
        size_t played = 0;
        esp_err_t err = tts_try(&slots[i], text, sink, user, &played);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        if (played > 0) {
            return err;
        }
        last = err;
    }

    /* Pass 2: one last try on the preferred slot. */
    {
        size_t played = 0;
        esp_err_t err = tts_try(&slots[0], text, sink, user, &played);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        if (played == 0) {
            last = err;
        }
    }

    ESP_LOGE(TAG, "every TTS backend failed");
    return last;
}


/*
 * Placeholder detection for the pre-flight check in main.c.
 *
 * Only the active backend's key matters, and both are reported so a half-filled
 * configuration is obvious rather than silently accepted. The iFlytek trio is
 * still checked even when SiliconFlow is active, because switching back is a
 * menuconfig change and the user should learn now, not after the switch, that
 * those three fields were never filled in.
 */
bool speech_backend_key_is_placeholder(void)
{
    bool placeholder = false;

#if CONFIG_STT_BACKEND_SILICONFLOW || CONFIG_TTS_BACKEND_SILICONFLOW
    if (strstr(app_cfg_siliconflow_key(), "REPLACE_ME") != NULL) {
        ESP_LOGW(TAG, "SiliconFlow API key is still the placeholder value");
        placeholder = true;
    }
#endif

#if CONFIG_STT_BACKEND_IFLYTEK || CONFIG_TTS_BACKEND_IFLYTEK
    if (strstr(app_cfg_iflytek_app_id(), "REPLACE_ME") != NULL ||
        strstr(app_cfg_iflytek_api_key(), "REPLACE_ME") != NULL ||
        strstr(app_cfg_iflytek_api_secret(), "REPLACE_ME") != NULL) {
        ESP_LOGW(TAG, "iFlytek credentials are still placeholder values");
        placeholder = true;
    }
#endif

    return placeholder;
}
