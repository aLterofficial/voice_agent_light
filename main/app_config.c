/*
 * Thin accessors over the Kconfig values.
 *
 * Having one place that reads CONFIG_* keeps the rest of the code free of
 * preprocessor noise and makes it obvious which strings are secrets.
 */

#include <string.h>

#include "app_internal.h"
#include "sdkconfig.h"

const char *app_cfg_wifi_ssid(void) { return CONFIG_WIFI_SSID; }
const char *app_cfg_wifi_password(void) { return CONFIG_WIFI_PASSWORD; }

/*
 * The second slot exists for a phone hotspot, so the device works both in the
 * lab and at a venue without a recompile. It is optional: the sentinel default
 * means "unused" and the WiFi manager skips it.
 */
const char *app_cfg_wifi_ssid2(void) { return CONFIG_WIFI_SSID_2; }
const char *app_cfg_wifi_password2(void) { return CONFIG_WIFI_PASSWORD_2; }

const char *app_cfg_deepseek_key(void) { return CONFIG_DEEPSEEK_API_KEY; }
const char *app_cfg_deepseek_base_url(void) { return CONFIG_DEEPSEEK_BASE_URL; }
const char *app_cfg_deepseek_model(void) { return CONFIG_DEEPSEEK_MODEL; }
const char *app_cfg_deepseek_system_prompt(void) { return CONFIG_DEEPSEEK_SYSTEM_PROMPT; }
int app_cfg_deepseek_max_tokens(void) { return CONFIG_DEEPSEEK_MAX_TOKENS; }

const char *app_cfg_iflytek_app_id(void) { return CONFIG_IFLYTEK_APP_ID; }
const char *app_cfg_iflytek_api_key(void) { return CONFIG_IFLYTEK_API_KEY; }
const char *app_cfg_iflytek_api_secret(void) { return CONFIG_IFLYTEK_API_SECRET; }
const char *app_cfg_iflytek_stt_host(void) { return CONFIG_IFLYTEK_STT_HOST; }
const char *app_cfg_iflytek_tts_host(void) { return CONFIG_IFLYTEK_TTS_HOST; }
const char *app_cfg_iflytek_tts_voice(void) { return CONFIG_IFLYTEK_TTS_VOICE; }
int app_cfg_iflytek_tts_speed(void) { return CONFIG_IFLYTEK_TTS_SPEED; }

const char *app_cfg_siliconflow_base_url(void) { return CONFIG_SILICONFLOW_BASE_URL; }
const char *app_cfg_siliconflow_key(void) { return CONFIG_SILICONFLOW_API_KEY; }
const char *app_cfg_siliconflow_stt_model(void) { return CONFIG_SILICONFLOW_STT_MODEL; }
const char *app_cfg_siliconflow_tts_model(void) { return CONFIG_SILICONFLOW_TTS_MODEL; }
const char *app_cfg_siliconflow_tts_voice(void) { return CONFIG_SILICONFLOW_TTS_VOICE; }
const char *app_cfg_siliconflow_tts_speed(void) { return CONFIG_SILICONFLOW_TTS_SPEED; }
const char *app_cfg_siliconflow_tts_gain(void) { return CONFIG_SILICONFLOW_TTS_GAIN; }

int app_cfg_audio_sample_rate(void) { return CONFIG_AUDIO_SAMPLE_RATE; }
int app_cfg_audio_out_sample_rate(void) { return CONFIG_AUDIO_OUT_SAMPLE_RATE; }

bool app_cfg_tts_local_enabled(void) { return CONFIG_TTS_LOCAL_ENABLE; }
const char *app_cfg_tts_local_base_url(void) { return CONFIG_TTS_LOCAL_BASE_URL; }
const char *app_cfg_tts_local_voice(void) { return CONFIG_TTS_LOCAL_VOICE; }
int app_cfg_audio_record_seconds(void) { return CONFIG_AUDIO_RECORD_SECONDS; }
int app_cfg_audio_silence_ms(void) { return CONFIG_AUDIO_SILENCE_MS; }
int app_cfg_audio_silence_threshold(void) { return CONFIG_AUDIO_SILENCE_THRESHOLD; }

bool app_cfg_secrets_are_placeholders(void)
{
    /* strstr rather than strcmp: the defaults are "sk-REPLACE_ME" and
     * "REPLACE_ME_APPID" etc, so a prefix match keeps this simple. */
    if (strstr(app_cfg_deepseek_key(), "REPLACE_ME") != NULL) {
        return true;
    }
    /*
     * Speech credentials: only the ACTIVE backend is required to be filled in.
     *
     * Checking both unconditionally would make an iFlytek-free setup look
     * unconfigured and refuse to start, which is exactly the configuration a
     * SiliconFlow user has - the other provider's fields are deliberately left
     * as placeholders.
     */
#if CONFIG_STT_BACKEND_SILICONFLOW || CONFIG_TTS_BACKEND_SILICONFLOW
    if (strstr(app_cfg_siliconflow_key(), "REPLACE_ME") != NULL) {
        return true;
    }
#endif

#if CONFIG_STT_BACKEND_IFLYTEK || CONFIG_TTS_BACKEND_IFLYTEK
    if (strstr(app_cfg_iflytek_app_id(), "REPLACE_ME") != NULL) {
        return true;
    }
    if (strstr(app_cfg_iflytek_api_key(), "REPLACE_ME") != NULL) {
        return true;
    }
    if (strstr(app_cfg_iflytek_api_secret(), "REPLACE_ME") != NULL) {
        return true;
    }
#endif

    /*
     * WiFi is satisfied if EITHER slot is filled in. Filling only slot 2 -
     * the hotspot - is a legitimate setup for a venue, and must not be
     * treated as "not configured".
     */
    const bool slot1_used = (strcmp(app_cfg_wifi_ssid(), "YOUR_WIFI_SSID") != 0);
    const bool slot2_used = (strcmp(app_cfg_wifi_ssid2(), "YOUR_WIFI_SSID_2") != 0);
    if (!slot1_used && !slot2_used) {
        return true;
    }

    return false;
}
