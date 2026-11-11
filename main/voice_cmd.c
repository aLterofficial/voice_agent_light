/*
 * Local voice commands.
 *
 * The voice assistant's normal path is: record -> speech-to-text -> DeepSeek ->
 * text-to-speech. That is the right shape for questions, but it is the wrong
 * shape for "turn the light on":
 *
 *   * it costs a DeepSeek round trip (and tokens) to answer something the
 *     device already knows how to do;
 *   * it adds two network round trips of latency before anything moves;
 *   * it fails outright with no network - and this device is meant to throw a
 *     wall switch, which is exactly the thing you still want to work when the
 *     WiFi is down.
 *
 * So recognised phrases are handled here, locally, before the DeepSeek call.
 * The reply is spoken through the same TTS path, so the experience is
 * unchanged from the user's side.
 *
 * MATCHING
 * --------
 * Plain substring matching on the recognised text. Deliberately not a real
 * intent parser: the STT output for these short phrases is stable, and a
 * keyword match is something that can be read and predicted. Anything that does
 * not match falls through to DeepSeek untouched.
 *
 * Order matters - see voice_cmd_handle().
 */

#include "app_internal.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <string.h>

static const char *TAG = "voicecmd";

#if CONFIG_SERVO_ENABLE

/* Case-insensitive substring search.
 *
 * Chinese has no case, but the same table also carries English phrases ("on",
 * "off") for testing, and those do. Doing the comparison by hand avoids
 * depending on strcasestr(), which is a GNU extension and not declared by
 * newlib in the default configuration. */
static bool contains_ci(const char *haystack, const char *needle)
{
    if (!haystack || !needle || !*needle) {
        return false;
    }

    const size_t nlen = strlen(needle);
    for (const char *p = haystack; *p; p++) {
        size_t i = 0;
        while (i < nlen && p[i]) {
            char a = p[i];
            char b = needle[i];
            if (a >= 'A' && a <= 'Z') { a = (char)(a - 'A' + 'a'); }
            if (b >= 'A' && b <= 'Z') { b = (char)(b - 'A' + 'a'); }
            if (a != b) {
                break;
            }
            i++;
        }
        if (i == nlen) {
            return true;
        }
    }
    return false;
}

/* Speak a short local reply. Falls back to the serial log when TTS is
 * unavailable, so the command still leaves a trace.
 *
 * Goes through tts_synthesize() rather than naming a provider directly: this
 * used to call iflytek_tts_synthesize() explicitly, which meant the reply to a
 * local command ignored CONFIG_TTS_BACKEND_* entirely and kept using iFlytek
 * even with SiliconFlow selected. With the iFlytek credentials left as
 * placeholders that failed silently - the switch still moved, but the
 * confirmation was only ever printed, never spoken. */
static void say(const char *text)
{
    void *pcm = NULL;
    size_t pcm_len = 0;

    if (tts_synthesize(text, &pcm, &pcm_len) == ESP_OK && pcm && pcm_len) {
        audio_output_play(pcm, pcm_len);
        free(pcm);
    } else {
        printf("\n[assistant] %s\n\n", text);
    }
}

#endif /* CONFIG_SERVO_ENABLE */

bool voice_cmd_handle(const char *text)
{
#if !CONFIG_SERVO_ENABLE
    (void)text;
    return false;
#else
    if (!text || !*text) {
        return false;
    }

    /*
     * ORDER MATTERS within each pair: check the longer, more specific phrase
     * first so a leading verb ("打开..." / "关闭...") cannot be matched by a
     * broader rule. And the whole "off" group is checked before the "on" group,
     * because "关灯" and "开灯" both end in the same character.
     *
     * NO bare "on"/"off" entries here. A one- or two-letter keyword matches
     * almost any sentence heard over the microphone ("...on the table..."), so
     * the device would throw the switch on unrelated speech. English phrases
     * are kept multi-word for that reason.
     */
    struct cmd {
        const char *phrase;
        const char *reply;
        bool        to_b;        /* false: position A, true: position B */
    };

    static const struct cmd k_cmds[] = {
        { "关闭灯",   "好的，灯已关闭",   true  },
        { "把灯关",   "好的，灯已关闭",   true  },
        { "关灯",     "好的，灯已关闭",   true  },
        { "turn off", "Okay, light off",  true  },

        { "打开灯",   "好的，灯已打开",   false },
        { "把灯开",   "好的，灯已打开",   false },
        { "开灯",     "好的，灯已打开",   false },
        { "turn on",  "Okay, light on",   false },
    };

    for (size_t i = 0; i < sizeof(k_cmds) / sizeof(k_cmds[0]); i++) {
        if (!contains_ci(text, k_cmds[i].phrase)) {
            continue;
        }

        /* Already there? Say so without moving - a servo that slams into a
         * switch it is already holding just wears the gears out. */
        const int target = k_cmds[i].to_b ? CONFIG_SERVO_POS_B_DEG
                                          : CONFIG_SERVO_POS_A_DEG;
        const int now = servo_get_angle();
        const int mid = (CONFIG_SERVO_POS_A_DEG + CONFIG_SERVO_POS_B_DEG) / 2;
        const bool currently_b = (now >= mid);

        ESP_LOGI(TAG, "\"%s\" matched \"%s\" (at %d deg, target %d deg)",
                 text, k_cmds[i].phrase, now, target);

        if (currently_b == k_cmds[i].to_b) {
            ESP_LOGI(TAG, "already in that position - not moving");
            say(k_cmds[i].reply);
            return true;
        }

        esp_err_t err = k_cmds[i].to_b ? servo_to_b() : servo_to_a();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "move failed: %s", esp_err_to_name(err));
            say("抱歉，开关没有动");
            return true;        /* handled - do not also ask DeepSeek */
        }

        ESP_LOGI(TAG, "moved to %d deg", servo_get_angle());
        say(k_cmds[i].reply);
        return true;
    }

    return false;               /* not a local command; let DeepSeek answer */
#endif
}
