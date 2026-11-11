/*
 * ESP32-S3 voice assistant - main application.
 *
 * Turn loop:
 *
 *   1. wait for the wake word (WakeNet, or energy VAD in bring-up mode)
 *   2. record the utterance from the INMP441 (I2S), with silence detection
 *   3. POST the PCM to iFlytek STT  -> text
 *   4. POST the text to DeepSeek    -> reply text
 *   5. POST the reply to iFlytek TTS -> PCM
 *   6. play the PCM through the MAX98357A (I2S)
 *
 * Hardware:
 *   ESP32-S3 (QFN56) rev v0.2, 240 MHz, 16 MB flash, 8 MB PSRAM
 *   INMP441     I2S digital microphone
 *   MAX98357A   I2S class-D amplifier
 *   CH343       USB-UART bridge on COM5
 */

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_internal.h"
#include "driver/gpio.h"
#include "esp_app_desc.h"
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_system.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include <strings.h>
#include <sys/time.h>
#include "esp_sntp.h"
#include "esp_timer.h"
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if CONFIG_FORCE_DISABLE_BROWNOUT
/*
 * esp_brownout_disable() exists in libesp_hw_support but is not part of any
 * public header, so it has to be declared by hand.
 *
 * Why this is needed at all: CONFIG_ESP_BROWNOUT_DET=n only prevents IDF from
 * arming the detector during startup. The ESP32-S3 re-arms the PMU comparator
 * as part of the RF power-up path, so a detector that was never explicitly
 * disabled still fires the moment the WiFi radio's current draw pulls the
 * supply down. Calling esp_brownout_disable() is the only way to actually
 * turn it off.
 *
 * See the CONFIG_FORCE_DISABLE_BROWNOUT help text for the risks.
 */
extern void esp_brownout_disable(void);
#endif

static const char *TAG = "assistant";

#define HISTORY_MAX 8

static chat_msg_t s_history[HISTORY_MAX];
static size_t s_history_len = 0;

/* Scratch buffers, allocated once from PSRAM. */
static int16_t *s_rec_buf = NULL;
static size_t s_rec_cap = 0;

/* ------------------------------------------------------------------------ */
/* Conversation memory                                                       */
/* ------------------------------------------------------------------------ */

static void history_clear(void)
{
    for (size_t i = 0; i < s_history_len; i++) {
        free((void *)s_history[i].content);
    }
    s_history_len = 0;
}

static void history_push(const char *role, const char *content)
{
    if (s_history_len >= HISTORY_MAX) {
        /* drop the oldest exchange, keeping system-level continuity by
         * removing a user/assistant pair */
        free((void *)s_history[0].content);
        free((void *)s_history[1].content);
        memmove(&s_history[0], &s_history[2], (s_history_len - 2) * sizeof(s_history[0]));
        s_history_len -= 2;
    }
    s_history[s_history_len].role = role; /* string literals, never freed */
    s_history[s_history_len].content = strdup(content);
    if (s_history[s_history_len].content != NULL) {
        s_history_len++;
    }
}

/* ------------------------------------------------------------------------ */
/* One conversational turn                                                   */
/* ------------------------------------------------------------------------ */

/*
 * Sentence-level synthesis.
 *
 * The reply used to be synthesised in one call, which meant the first syllable
 * was only heard after the whole clip had been generated - measured at 18 s for
 * a 6.6 s answer. Splitting at sentence boundaries and synthesising each one as
 * it becomes complete moves the first audio to roughly "one third of the
 * reply", which is what the listener actually perceives.
 *
 * This is the same idea the xiaozhi project exposes as TTS_STATE_SENTENCE_START,
 * done on the device because our three cloud APIs are separate request/response
 * calls with no orchestrator in between.
 */

/* Longest UTF-8 segment handed to one synthesis call. Long enough to keep the
 * prosody natural, short enough that a reply without punctuation still starts
 * speaking early. */
#define TTS_SEG_MAX 140

static size_t utf8_char_len(unsigned char c)
{
    if (c < 0x80) {
        return 1;
    }
    if ((c & 0xE0) == 0xC0) {
        return 2;
    }
    if ((c & 0xF0) == 0xE0) {
        return 3;
    }
    if ((c & 0xF8) == 0xF0) {
        return 4;
    }
    return 1;
}

/* 。 ! ？ ； . ! ? ; */
static bool ends_sentence(const char *p)
{
    const unsigned char *u = (const unsigned char *)p;

    if (u[0] == 0xE3 && u[1] == 0x80 && u[2] == 0x82) {
        return true;                                        /* 。 */
    }
    if (u[0] == 0xEF && u[1] == 0xBC &&
        (u[2] == 0x81 || u[2] == 0x9F || u[2] == 0x9B)) {
        return true;                                        /* ！ ？ ； */
    }
    return u[0] == '.' || u[0] == '!' || u[0] == '?' || u[0] == ';';
}

/* ， ,  - only used to break up an over-long run without full stops. */
static bool is_soft_break(const char *p)
{
    const unsigned char *u = (const unsigned char *)p;
    if (u[0] == 0xEF && u[1] == 0xBC && u[2] == 0x8C) {
        return true;                                        /* ， */
    }
    return u[0] == ',';
}

static bool is_space(char c)
{
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

/* Speak one trimmed segment. Returns the transport error, if any. */
static esp_err_t speak_segment(const char *text, size_t len)
{
    char buf[TTS_SEG_MAX + 8];

    while (len > 0 && is_space(text[0])) {
        text++;
        len--;
    }
    while (len > 0 && is_space(text[len - 1])) {
        len--;
    }
    if (len == 0) {
        return ESP_OK;
    }
    if (len > TTS_SEG_MAX) {
        len = TTS_SEG_MAX;
    }

    memcpy(buf, text, len);
    buf[len] = '\0';

    ESP_LOGI(TAG, "speak(%u): %s", (unsigned)len, buf);
    return tts_stream(buf, audio_output_sink, NULL);
}

/*
 * Speak the whole reply, one sentence at a time. The amplifier is unmuted once
 * and muted once, so the segments join seamlessly instead of clicking.
 */
static esp_err_t speak_reply(const char *reply)
{
    const size_t n = strlen(reply);
    esp_err_t err = audio_output_stream_begin();
    if (err != ESP_OK) {
        return err;
    }

    size_t seg_start = 0;
    size_t i = 0;
    int segments = 0;
    esp_err_t first_err = ESP_OK;
    esp_err_t last_err = ESP_OK;

    while (i < n) {
        const size_t adv = utf8_char_len((unsigned char)reply[i]);
        if (i + adv > n) {
            break;                       /* truncated UTF-8: stop cleanly */
        }
        i += adv;
        const char *ch = reply + i - adv;

        const bool hard = ends_sentence(ch);
        const bool full = (i - seg_start) >= TTS_SEG_MAX;
        const bool soft = is_soft_break(ch) && (i - seg_start) >= 60;
        if (!hard && !full && !soft) {
            continue;
        }

        err = speak_segment(reply + seg_start, i - seg_start);
        if (segments == 0) {
            first_err = err;             /* decides whether we fall back */
        }
        if (err != ESP_OK) {
            last_err = err;
            ESP_LOGW(TAG, "segment %d failed: %s", segments, esp_err_to_name(err));
            break;                       /* the session is already broken */
        }
        segments++;
        seg_start = i;
    }

    if (segments > 0 && last_err == ESP_OK && seg_start < n) {
        err = speak_segment(reply + seg_start, n - seg_start);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "trailing segment failed: %s", esp_err_to_name(err));
        }
    }

    esp_err_t end_err = audio_output_stream_end();
    if (first_err != ESP_OK) {
        return first_err;                /* nothing was heard: allow the fallback */
    }
    return end_err;
}


/*
 * The device has no RTC, so before SNTP the clock reads 1970. iFlytek's
 * WebSocket API signs every request with the current GMT date and the service
 * rejects anything outside a few minutes of its own clock - the handshake comes
 * back 401, which reads like a bad key but is a missing clock. SiliconFlow never
 * showed this because it authenticates with a Bearer token and no timestamp.
 *
 * Returns true once the clock is set.
 */

/*
 * Pull the clock from an HTTPS Date header.
 *
 * SNTP needs UDP 123, which phone hotspots routinely drop; HTTPS does not.
 * Every response carries a Date header, so one request is enough - the status
 * code is irrelevant (a 401 has the header too).
 */
static esp_err_t clock_head_event(esp_http_client_event_t *e)
{
    if (e->event_id == HTTP_EVENT_ON_HEADER && e->header_key != NULL &&
        e->header_value != NULL && strcasecmp(e->header_key, "Date") == 0) {
        struct tm tm;
        memset(&tm, 0, sizeof(tm));
        if (strptime(e->header_value, "%a, %d %b %Y %H:%M:%S", &tm) != NULL) {
            const time_t t = timegm(&tm);
            if (t > 1600000000) {
                struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
                settimeofday(&tv, NULL);
            }
        }
    }
    return ESP_OK;
}

static bool clock_sync_https(void)
{
    esp_http_client_config_t cfg = {
        .url = "https://api.siliconflow.cn/v1/models",
        .event_handler = clock_head_event,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 8000,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (c == NULL) {
        return false;
    }
    esp_http_client_set_method(c, HTTP_METHOD_GET);
    (void)esp_http_client_perform(c);
    esp_http_client_cleanup(c);

    time_t now = 0;
    time(&now);
    if ((long long)now > 1600000000LL) {
        ESP_LOGI(TAG, "clock set from HTTPS Date header");
        return true;
    }
    return false;
}

static bool wait_for_clock(int timeout_ms)
{
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "ntp.aliyun.com");
    esp_sntp_setservername(1, "pool.ntp.org");
    esp_sntp_init();

    const int step = 200;
    for (int waited = 0; waited < timeout_ms; waited += step) {
        time_t now = 0;
        time(&now);
        /* Anything past 2020 means SNTP has landed. */
        if ((long long)now > 1600000000LL) {
            struct tm tmv;
            gmtime_r(&now, &tmv);
            ESP_LOGI(TAG, "clock synced: %04d-%02d-%02d %02d:%02d:%02dZ",
                     tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
                     tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(step));
    }
    ESP_LOGW(TAG, "NTP unavailable after %d ms; the clock will be set from the first HTTP Date header",
             timeout_ms);
    return false;
}

//核心出装

static esp_err_t do_turn(void)
{
    /* --- 1. listen --- */
    led_status_set(LED_LISTENING);

    const int max_ms = app_cfg_audio_record_seconds() * 1000;
    size_t rec_len = 0;
    esp_err_t err = audio_input_record(
        s_rec_buf,                          /* 输出缓冲区 (int16_t* PCM) */
        s_rec_cap,                          /* 缓冲区容量，单位：字节 */
        &rec_len,                           /* 出参：实际录到的字节数 */
        max_ms,                             /* 硬性最长录音时间 */
        app_cfg_audio_silence_threshold(),  /* 静音判定门限 (RMS) */
        app_cfg_audio_silence_ms());        /* 静音持续多久就算说完 */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "recording failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Require a minimum amount of audio, otherwise the STT request is a
     * guaranteed waste of a round trip. */
    const size_t min_bytes = 16000 * 2 * 400 / 1000; /* 400 ms */
    if (rec_len < min_bytes) {
        ESP_LOGW(TAG, "only %u bytes captured - too short, ignoring",
                 (unsigned)rec_len);
        return ESP_ERR_INVALID_SIZE;
    }

    /* --- 2. speech to text --- */
    led_status_set(LED_THINKING);
    char *user_text = NULL;
    err = stt_recognize(s_rec_buf, rec_len, &user_text);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "STT failed: %s", esp_err_to_name(err));
        return err;
    }

    /*
     * --- 2b. local commands ---
     *
     * "turn the light on" is handled here and never reaches DeepSeek. Doing it
     * locally means the switch still works with no network, answers without two
     * round trips of latency, and costs no tokens. Anything unrecognised falls
     * through to the model exactly as before.
     *
     * The history is deliberately NOT updated for a local command: it is an
     * action, not a conversational turn, and feeding it into the history would
     * pollute the context the model sees for the next real question.
     */
    if (voice_cmd_handle(user_text)) {
        ESP_LOGI(TAG, "handled locally: \"%s\"", user_text);
        free(user_text);
        return ESP_OK;
    }

    /* --- 3. ask DeepSeek --- */
    history_push("user", user_text);
    free(user_text);

    char *reply = NULL;
    err = deepseek_client_chat(s_history, s_history_len, &reply);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DeepSeek failed: %s", esp_err_to_name(err));
        /* roll the failed user turn back out of the history */
        free((void *)s_history[s_history_len - 1].content);
        s_history_len--;
        return err;
    }
    history_push("assistant", reply);

    /* --- 4. speak --- */
    led_status_set(LED_SPEAKING);
    /*
     * Sentence by sentence, not one big call. The first sentence is synthesised
     * and played while the rest of the reply is still waiting its turn, so the
     * listener hears something after roughly a third of the wait instead of
     * after all of it. See speak_reply().
     */
    err = speak_reply(reply);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TTS failed: %s - printing the reply instead", esp_err_to_name(err));
        printf("\n[assistant] %s\n\n", reply);
    }

    free(reply);
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* Bring-up helpers                                                          */
/* ------------------------------------------------------------------------ */

static void log_banner(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    esp_chip_info_t chip;
    esp_chip_info(&chip);

    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "  %s  v%s", app->project_name, app->version);
    ESP_LOGI(TAG, "  IDF %s, built %s %s", app->idf_ver, app->date, app->time);
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "chip     : ESP32-S3 rev v%d.%d, %d cores",
             chip.revision / 100, chip.revision % 100, chip.cores);
    ESP_LOGI(TAG, "psram    : %u KB free of %u KB",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned)(esp_psram_get_size() / 1024));
    ESP_LOGI(TAG, "heap     : %" PRIu32 " KB internal free",
             esp_get_free_heap_size() / 1024);
    ESP_LOGI(TAG, "model    : %s", app_cfg_deepseek_model());
    ESP_LOGI(TAG, "wake     : %s",
#if CONFIG_WAKE_BACKEND_WAKENET
             "WakeNet (neural)"
#else
             "energy VAD (bring-up only)"
#endif
    );
    ESP_LOGI(TAG, "-------------------------------------------------");
}

static esp_err_t init_buffers(void)
{
    /* Aim for 30 s of 16 kHz 16-bit mono (960 KB), but accept whatever the
     * heap can actually provide.
     *
     * The full-size buffer only fits in PSRAM. That is fine on a healthy
     * board, but PSRAM on the ESP32-S3 is an octal device on the same MSPI
     * bus as the flash, so driving it hard costs current. When PSRAM is
     * unavailable - or when CONFIG_SPIRAM is off for a power experiment -
     * fall back to a shorter buffer in internal RAM rather than failing to
     * start.
     */
    const size_t want = 16000 * 2 * 30;  /* 30 s */
    const size_t min  = 16000 * 2 * 6;   /* 6 s */

    s_rec_buf = heap_caps_malloc(want, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_rec_buf != NULL) {
        s_rec_cap = want;
        ESP_LOGI(TAG, "recording buffer: %u KB in PSRAM (%.0f s)",
                 (unsigned)(want / 1024), 30.0);
        return ESP_OK;
    }

    /* No PSRAM, or not enough of it. Internal RAM it is. */
    const size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t ask = min;
    if (ask > internal_free / 2) {
        /* Never take more than half of what is free - WiFi needs internal RAM
         * too, and starving it causes its own set of failures. */
        ask = internal_free / 2;
    }
    /* Round down to a whole number of milliseconds of audio. */
    ask -= ask % (16000 * 2 / 1000);

    if (ask < 16000 * 2 / 2) {
        ESP_LOGE(TAG, "not enough RAM for even a short recording buffer "
                      "(free internal: %u KB)", (unsigned)(internal_free / 1024));
        return ESP_ERR_NO_MEM;
    }

    s_rec_buf = heap_caps_malloc(ask, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_rec_buf == NULL) {
        ESP_LOGE(TAG, "cannot allocate the recording buffer");
        return ESP_ERR_NO_MEM;
    }
    s_rec_cap = ask;
    ESP_LOGW(TAG, "recording buffer: %u KB in internal RAM (%.1f s) - PSRAM "
                  "unavailable, so recordings are shorter",
             (unsigned)(ask / 1024),
             (double)ask / (16000 * 2));
    return ESP_OK;
}

void app_main(void)
{
    log_banner();

#if CONFIG_RAIL_MONITOR
    /*
     * Diagnostic firmware takes over completely: no LED, no audio, no radio -
     * just the rail voltage on a local display. Deliberately first, so nothing
     * else is drawing current while the rail is being measured.
     */
    rail_monitor_run();
    /* rail_monitor_run() never returns */
#endif

    if (app_cfg_secrets_are_placeholders()) {
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "*** CONFIGURATION REQUIRED ***");
        ESP_LOGE(TAG, "Some credentials are still placeholders. Run:");
        ESP_LOGE(TAG, "    idf.py menuconfig");
        ESP_LOGE(TAG, "and fill in:");
        ESP_LOGE(TAG, "    Voice assistant configuration -> Network");
        ESP_LOGE(TAG, "    Voice assistant configuration -> DeepSeek");
        ESP_LOGE(TAG, "    Voice assistant configuration -> iFlytek (讯飞)");
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "The device will idle with a red LED until then.");
        ESP_LOGE(TAG, "");
        led_status_init();
        led_status_set(LED_ERROR);
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_ERROR_CHECK(led_status_init());
    led_status_set(LED_WIFI);

    /*
     * Bring the serial console up early: if the board cannot join the
     * network at the venue, 'w' (scan) is the command that answers why, and
     * it has to be available even when the rest of the startup is blocked
     * waiting for a link.
     */
    console_cmd_init();

#if CONFIG_SERVO_ENABLE
    /* Before the radio, deliberately: the servo's first move is its biggest
     * current spike, and doing it while WiFi is also powering up is how the
     * board gets reset. See servo.c and SHOPPING-AND-WIRING.md. */
    ESP_ERROR_CHECK(servo_init());
#endif

#if CONFIG_SERVO_TEST_MODE
    /*
     * Bench mode for the servo: no WiFi, no audio - just the console, so the
     * switch's mechanics and endpoints can be found without the board
     * resetting every 1.4 s underneath the person doing the measuring.
     *
     * Nothing is moved automatically: the user drives it with a/b/t/,/. and
     * watches for the arm stalling or buzzing, which is what sets the
     * endpoint values in menuconfig.
     */
    ESP_LOGW(TAG, "*** SERVO TEST MODE - the radio and audio are NOT started ***");
    ESP_LOGW(TAG, "*** Use the console: h for help, a/b/t to move, ',' '.' to jog ***");
    ESP_LOGW(TAG, "*** A buzzing servo is stalled: back off and use that angle "
                  "as the endpoint ***");
    /*
     * Report what is on the I2C bus even in this mode.
     *
     * The rail monitor is the firmware that drives the OLED, so in servo bench
     * mode the display is dark no matter how it is wired - which makes "is the
     * screen dead or is this the wrong firmware?" impossible to answer by
     * looking at it. Probing here answers it.
     */
    {
        int sda = CONFIG_I2C_SDA_GPIO, scl = CONFIG_I2C_SCL_GPIO;
        uint8_t addr = 0;
        if (i2c_scan_find_report(CONFIG_I2C_PROBE_ADDR, sda, scl,
                                 &sda, &scl, &addr, 3000)) {
            ESP_LOGW(TAG, "I2C device present at 0x%02X (SDA=GPIO%d SCL=GPIO%d). "
                          "NOTE: this firmware does not drive the display - "
                          "switch to the RAIL MONITOR firmware to see it.",
                     addr, sda, scl);
        } else {
            ESP_LOGE(TAG, "no I2C device found - check SDA/SCL/VCC/GND on the OLED");
        }
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "servo test mode: position %d deg", servo_get_angle());
    }
#endif

    ESP_ERROR_CHECK(init_buffers());
    ESP_ERROR_CHECK(audio_output_init());
    ESP_ERROR_CHECK(audio_input_init());

    /*
     * Reference tone. This is the only audio the device makes that does not
     * come from a cloud service: a pure 440 Hz sine at a known level. If the
     * speaker sounds clean here but noisy on a reply, the playback path is
     * fine and the noise is in the TTS audio. If this is noisy too, the I2S
     * configuration itself is at fault.
     *
     * It used to sit at 9 % amplitude, which is around 0.02 W on a 3 W speaker
     * - inaudible in a normal room, so it could not serve as a reference.
     */
    led_status_set(LED_SPEAKING);
    {
        /* 300 ms 440 Hz sine at 16 kHz, about -4 dBFS */
        const int sr = 16000;
        const int n = sr * 300 / 1000;
        int16_t *chime = malloc(n * sizeof(int16_t));
        if (chime != NULL) {
            ESP_LOGI(TAG, "reference tone: 440 Hz, 300 ms, -4 dBFS");
            for (int i = 0; i < n; i++) {
                double t = (double)i / sr;
                /* fade in/out so there is no click */
                double env = 1.0;
                if (i < sr / 50) {
                    env = (double)i / (sr / 50);
                } else if (i > n - sr / 50) {
                    env = (double)(n - i) / (sr / 50);
                }
                chime[i] = (int16_t)(20000 * env * sin(2 * M_PI * 440 * t));
            }
            audio_output_play(chime, n * sizeof(int16_t));
            free(chime);
        }
    }

    /*
     * Silence test, straight after the tone.
     *
     * The amplifier is enabled for two seconds with nothing but digital zeros.
     * Any hiss heard in this window is generated by the amplifier itself or by
     * its supply - it cannot be data corruption, because the data is all zeros.
     * That separates "the I2S path is dirty" from "the analogue side is dirty",
     * which have different fixes.
     */
    {
        const int n = 16000 * 2;
        int16_t *zeros = calloc((size_t)n, sizeof(int16_t));
        if (zeros != NULL) {
            ESP_LOGI(TAG, "silence test: 2 s of digital zeros, amp enabled");
            audio_output_play(zeros, n * sizeof(int16_t));
            free(zeros);
            ESP_LOGI(TAG, "silence test: done");
        }
    }

    led_status_set(LED_IDLE);

#if CONFIG_FORCE_DISABLE_BROWNOUT
    /* Must happen before the radio starts, because that is what re-arms the
     * comparator. Placed here rather than at the top of app_main so the
     * startup banner and peripheral init still run under the detector's
     * protection - if the supply cannot even manage those, we want the reset
     * rather than silent misbehaviour. */
    esp_brownout_disable();
    ESP_LOGW(TAG, "brownout detector FORCED OFF (workaround) - the supply "
                  "is not actually fixed, see CONFIG_FORCE_DISABLE_BROWNOUT");
#endif

#if CONFIG_SKIP_WIFI
    /* Bring-up diagnostic: stop here, before the radio is powered up. See the
     * SKIP_WIFI Kconfig help for how to read the result.
     *
     * The LED is driven from a plain count so the board can be checked with
     * no serial console attached at all - useful when the PC's USB port is
     * the thing under suspicion.
     *
     * A steady blue LED that survives for minutes means PSRAM, both I2S
     * peripherals and the LED are all fine, and the only remaining variable
     * is the radio. */
    ESP_LOGW(TAG, "");
    ESP_LOGW(TAG, "*** SKIP_WIFI is enabled - the radio will NOT be started ***");
    ESP_LOGW(TAG, "*** A steady blue LED from here on means the board is     ***");
    ESP_LOGW(TAG, "*** stable without WiFi, so the supply is the culprit.     ***");
    ESP_LOGW(TAG, "");
    {
        uint32_t n = 0;
        while (1) {
            ESP_LOGI(TAG, "heartbeat %" PRIu32 "  (uptime %" PRIu32 " s, "
                          "heap %" PRIu32 " KB, min %" PRIu32 " KB)",
                     ++n, (uint32_t)(esp_timer_get_time() / 1000000),
                     esp_get_free_heap_size() / 1024,
                     heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
#else
    /*
     * Low power mode is applied in two phases, and the split matters.
     *
     * The radio's peak current is drawn during RF calibration, which runs
     * inside esp_wifi_init() - i.e. inside wifi_mgr_start(). The runtime TX
     * power API is not legal until esp_wifi_start() has returned, so a board
     * that cannot survive calibration resets before that call is ever
     * reached and the runtime cap never applies. That is why an earlier test
     * appeared to do nothing: the boot log stopped at the I2S init lines and
     * never reached the low-power banner.
     *
     * Phase 1 therefore has to be a compile-time setting
     * (CONFIG_ESP_PHY_MAX_TX_POWER, which feeds the calibration TX level),
     * and phase 2 - the runtime cap - is applied as soon as it is legal.
     */
    power_save_init_before_radio();

    /*
     * Shed everything that can be shed before the radio powers up: cap the
     * CPU, idle both I2S channels and blank the LED. Measured on this board
     * the reset lands inside esp_wifi_init() (see radio_bringup.c), which is
     * upstream of every runtime TX-power lever, so trimming the load that
     * rides along with the radio's inrush is the only software angle left.
     */
    radio_bringup_prepare();
    radio_bringup_cpu_before_radio();
    radio_bringup_mark("calling wifi_mgr_start()");

    wifi_mgr_start();

    radio_bringup_cpu_after_radio();
    radio_bringup_finish();
    power_save_init();

    wifi_mgr_wait_connected();

    if (!wait_for_clock(3000)) {
        clock_sync_https();
    }

#if CONFIG_TTS_SELFTEST
    /*
     * TTS self test: one known sentence through the *streaming* path.
     *
     * Streaming, not the buffered API, because the whole point is to exercise
     * what a real turn does - including the pre-roll and the per-block timing
     * that showed the network delivering unevenly. Running it at boot means
     * the pipeline can be measured without anyone having to speak.
     */
    {
        static const char kSelftestText[] = "语音合成测试，一二三四五六七八九十。";
        time_t now = 0;
        time(&now);
        if ((long long)now < 1600000000LL) {
            ESP_LOGW(TAG, "tts self test skipped: clock not set yet "
                          "(Signed requests would be rejected)");
        } else {
            ESP_LOGI(TAG, "tts self test: \"%s\"", kSelftestText);
            const int64_t t0 = esp_timer_get_time();

            esp_err_t e = audio_output_stream_begin();
            if (e == ESP_OK) {
                e = tts_stream(kSelftestText, audio_output_sink, NULL);
            }
            const int64_t t1 = esp_timer_get_time();
            audio_output_stream_end();

            if (e == ESP_OK) {
                ESP_LOGI(TAG, "tts self test: streamed and played in %lld ms",
                         (long long)((t1 - t0) / 1000));
            } else {
                ESP_LOGE(TAG, "tts self test FAILED after %lld ms: %s",
                         (long long)((t1 - t0) / 1000), esp_err_to_name(e));
            }
        }
    }
#endif

    /*
     * No failure path here on purpose. wifi_mgr_wait_connected() returns only
     * once a network is up; the WiFi manager retries forever with backoff, so
     * a hotspot that is switched on late is still picked up. The previous
     * version parked the device in an error loop at this point, which meant a
     * venue could never recover without a manual reset.
     */

    ESP_ERROR_CHECK(deepseek_client_init());
    ESP_ERROR_CHECK(wake_word_init());
#endif


    led_status_set(LED_IDLE);
    ESP_LOGI(TAG, "ready - say the wake word");
#if CONFIG_BOOT_BUTTON_TRIGGER
    ESP_LOGI(TAG, "(or hold BOOT / GPIO0 to start a turn manually)");
#endif

    /* ------------------------------------------------------------------ */
    /* Main loop                                                          */
    /* ------------------------------------------------------------------ */
    while (1) {
        /*
         * A phone hotspot will sometimes drop the link - it sleeps, the
         * user walks away and back, or the AP reboots. Running a turn with
         * no network just burns a few seconds and fails in STT, so wait here
         * for the manager to re-establish the link first.
         *
         * The wait is bounded and the loop re-checks, which keeps the LED
         * (and the console) alive instead of blocking forever.
         */
        if (!wifi_mgr_is_connected()) {
            led_status_set(LED_WIFI);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        led_status_set(LED_IDLE);

        esp_err_t err = wake_word_wait(0); /* 0 = wait forever */
        if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        err = do_turn();
        if (err != ESP_OK) {
            led_status_blink_error();
            vTaskDelay(pdMS_TO_TICKS(1200));
        }

        /*
         * Settle time. The speaker sits next to the microphone, so the tail of
         * our own reply is still in the room - and in the I2S DMA - when the
         * loop comes back round. 300 ms was short enough that the device would
         * transcribe itself and answer its own sentence, which burns an STT,
         * an LLM and a TTS call each time and floods the service queue.
         */
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
