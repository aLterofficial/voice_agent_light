/*
 * Voice assistant - shared declarations.
 *
 * Pipeline:
 *
 *   INMP441 --I2S--> audio_input --> [wake word] --> iFlytek STT
 *        --> DeepSeek chat --> iFlytek TTS --> audio_output --I2S--> MAX98357A
 *
 * All audio is 16 kHz / 16-bit / mono PCM, which is what both the iFlytek
 * services and ESP-SR expect.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* LED status                                                                */
/* ------------------------------------------------------------------------ */

typedef enum {
    LED_OFF = 0,
    LED_IDLE,       /* dim blue    - waiting for the wake word        */
    LED_LISTENING,  /* green pulse - recording the user's speech      */
    LED_THINKING,   /* amber spin  - STT / DeepSeek round trip        */
    LED_SPEAKING,   /* cyan        - playing the TTS reply            */
    LED_ERROR,      /* red         - something failed                 */
    LED_WIFI,       /* purple      - connecting to WiFi               */
} led_state_t;

esp_err_t led_status_init(void);
void led_status_set(led_state_t state);
void led_status_blink_error(void);

/*
 * Blank the LED for the radio bring-up window and restore the previous
 * pattern afterwards. See radio_bringup.c and led_status.c.
 */
void led_status_quiet(void);
void led_status_unquiet(void);

/* ------------------------------------------------------------------------ */
/* WiFi                                                                      */
/* ------------------------------------------------------------------------ */

esp_err_t wifi_mgr_start(void);
bool wifi_mgr_is_connected(void);

/*
 * Block until the first connection succeeds. Never returns on failure: the
 * reconnect loop retries forever, which is what an unattended demo needs.
 */
void wifi_mgr_wait_connected(void);

/*
 * Blocking scan, then print every visible AP with its channel, RSSI and
 * security mode. Configured SSIDs are marked, and a warning is printed for
 * any configured SSID that did not appear - the usual cause being a phone
 * hotspot left on 5 GHz.
 *
 * Intended for the 'w' console command, i.e. for diagnosing a venue where
 * the device will not connect.
 */
void wifi_mgr_scan_and_report(void);

/* Copy the SSIDs actually in use into `out` (NULL-terminated, up to `max`). */
void wifi_mgr_describe_candidates(const char **out, int max);

/* Drop the link and retry immediately, skipping any pending backoff. */
void wifi_mgr_reconnect_now(void);

/* ------------------------------------------------------------------------ */
/* Serial console commands (console_cmd.c)                                   */
/* ------------------------------------------------------------------------ */

/* Install the console UART driver and start the command task. */
esp_err_t console_cmd_init(void);

/* ------------------------------------------------------------------------ */
/* Power management (power_save.c)                                           */
/* ------------------------------------------------------------------------ */

/*
 * Apply the low-power settings, if CONFIG_LOW_POWER_MODE is set. Must be
 * called AFTER the WiFi interface is started, because the TX power API
 * rejects calls made before esp_wifi_start().
 *
 * A no-op when low power mode is disabled.
 */
esp_err_t power_save_init(void);

/*
 * Pre-radio half of the above. Call BEFORE wifi_mgr_start().
 *
 * RF calibration runs inside esp_wifi_init() and draws the current spike that
 * trips the brownout detector; the runtime TX power API cannot be used until
 * esp_wifi_start(), by which point a supply-starved board has already reset.
 * This reports the compile-time PHY TX power setting that calibration uses so
 * a misconfiguration is visible in the boot log rather than silent.
 */
esp_err_t power_save_init_before_radio(void);

/* Re-apply the settings after a link reset. Safe to call at any time. */
void power_save_reapply(void);

/* ------------------------------------------------------------------------ */
/* Radio bring-up: current shedding (radio_bringup.c)                        */
/* ------------------------------------------------------------------------ */

/*
 * Everything that switches at the same instant as the radio's MAC/PHY
 * power-up is switched off first, so the burst sees the smallest baseline
 * load. Call BEFORE wifi_mgr_start().
 *
 * Measured 2026-10-02: the reset lands inside esp_wifi_init(), which is
 * upstream of esp_wifi_start() and therefore upstream of every runtime TX
 * power lever. These functions attack the only thing that is still reachable
 * from software - the load that rides along with the radio's inrush.
 *
 * Both are compiled to no-ops unless the CONFIG_BRINGUP_* options are set.
 */
void radio_bringup_prepare(void);

/* Cap the CPU (CONFIG_BRINGUP_LOW_CPU) for the radio power-up window. */
void radio_bringup_cpu_before_radio(void);

/* Undo radio_bringup_cpu_before_radio(). Call once the radio is up. */
void radio_bringup_cpu_after_radio(void);

/* Restore the LED and both I2S channels. Call once the radio is up. */
void radio_bringup_finish(void);

/*
 * Print one bring-up marker: elapsed time since prepare() plus free internal
 * heap. Compiled out unless a diagnostic switch is on. Safe to call at any
 * point; used to bracket steps that are too fine-grained for the markers in
 * wifi_mgr.c.
 */
void radio_bringup_mark(const char *what);

/* ------------------------------------------------------------------------ */
/* Rail monitor (rail_monitor.c)                                             */
/* ------------------------------------------------------------------------ */

/*
 * Diagnostic firmware: samples the 3.3V rail through an ADC pin and shows the
 * current / 5 s minimum / lowest-since-boot voltage plus the reset reason and
 * a reset counter on an I2C OLED.
 *
 * Exists because the USB serial bridge is on the rail under test: when the
 * rail collapses the port disappears, so the instrument cannot be the host.
 * Called instead of the assistant's normal startup when
 * CONFIG_RAIL_MONITOR is set; never returns.
 */
void rail_monitor_run(void);

/* ------------------------------------------------------------------------ */
/* I2C bus discovery (i2c_scan.c)                                            */
/* ------------------------------------------------------------------------ */

/*
 * Find which GPIO pair an I2C module is wired to.
 *
 * Tries the supplied pins first; if nothing answers there, sweeps every
 * plausible pair. Strapping pins (0/3/45/46), native USB (19/20), flash and
 * PSRAM (26..37) and the console UART (43/44) are never probed.
 *
 * Why this exists: the pins are a free choice on a breadboard, and finding out
 * by recompiling costs a flash cycle per guess - and makes a wiring mistake look
 * like a firmware bug.
 *
 * @param start_addr  address to try first (0 = probe the whole range)
 * @param start_sda   pin to try first, or -1 to skip straight to the sweep
 * @param start_scl   pin paired with start_sda
 * @param timeout_ms  budget for the sweep phase
 * @return true if a device was found; the working pins and address are written
 *         through the out-parameters.
 */
bool i2c_scan_find_report(uint8_t start_addr, int start_sda, int start_scl,
                          int *sda_out, int *scl_out, uint8_t *addr_out,
                          int timeout_ms);

/* ------------------------------------------------------------------------ */
/* Servo (servo.c)                                                           */
/* ------------------------------------------------------------------------ */

/*
 * RC servo output for throwing a panel switch. The control signal is generated
 * by the LEDC peripheral in hardware, so it stays accurate with WiFi running
 * and costs no CPU time once a position is set.
 *
 * The servo's POWER is not part of this API: it must come from the 5V rail on
 * its own branch with its own bulk capacitor. Feeding a servo from 3V3, or
 * through the board's 5V header, resets the board when it moves - see
 * SHOPPING-AND-WIRING.md.
 *
 * Every function is a no-op returning ESP_OK when CONFIG_SERVO_ENABLE is unset,
 * so callers need no #ifdefs.
 */
esp_err_t servo_init(void);
int servo_get_angle(void);

/* Jump straight to an angle. Fast, and the biggest current spike - prefer
 * servo_move_smooth() for anything mechanical. */
esp_err_t servo_move_to(int deg);

/* Ramped move in CONFIG_SERVO_STEP_DEG steps, CONFIG_SERVO_STEP_MS apart. */
esp_err_t servo_move_smooth(int deg);

/* Move to the opposite switch position (A <-> B), ramped. */
esp_err_t servo_toggle(void);
esp_err_t servo_to_a(void);
esp_err_t servo_to_b(void);

/*
 * Stop sending pulses: no holding torque and no standing current. Worth using
 * when the switch holds itself, because a servo holding against a spring is a
 * constant load on a supply that is already marginal.
 */
esp_err_t servo_release(void);

/*
 * Angle stored by a previous run, or `fallback` when there is nothing usable.
 *
 * NOT called from servo_init(): reading it initialises NVS, and on this board
 * that browns out during boot. Call it once the supply is fixed (see servo.c).
 */
int servo_recall_angle(int fallback);

/* ------------------------------------------------------------------------ */
/* Local voice commands (voice_cmd.c)                                        */
/* ------------------------------------------------------------------------ */

/*
 * Handle phrases the device can act on by itself, before the text is sent to
 * DeepSeek.
 *
 * Call this with the speech-to-text result. Returns true when the text was a
 * recognised local command and has been acted on (and spoken) - in that case
 * the caller must NOT also send it to DeepSeek. Returns false for everything
 * else, which is the common case: ordinary questions fall through to the model
 * unchanged.
 *
 * Deliberately local: throwing a switch should not need a network round trip,
 * should not cost tokens, and has to keep working when the WiFi is down.
 *
 * Always returns false when CONFIG_SERVO_ENABLE is unset, so callers need no
 * #ifdefs.
 */
bool voice_cmd_handle(const char *text);

/* ------------------------------------------------------------------------ */
/* Audio input - INMP441 over I2S                                            */
/* ------------------------------------------------------------------------ */

typedef struct {
    /* filled in once by audio_input_init */
    size_t sample_rate;
    size_t bytes_per_sample; /* 2 for 16-bit */
} audio_format_t;

esp_err_t audio_input_init(void);

/*
 * Idle / re-enable the RX channel. Used only by radio_bringup.c to remove the
 * microphone's clock from the load while the radio powers up; the channel
 * itself is preserved so resume is exact.
 */
esp_err_t audio_input_stop(void);
esp_err_t audio_input_resume(void);
const audio_format_t *audio_input_format(void);

/*
 * Record one utterance.
 *
 * Blocks until either `max_ms` elapses or the signal has stayed below
 * `silence_rms` for `silence_ms` after having first exceeded it.
 *
 * @param out_buf         caller-owned destination (16-bit mono PCM)
 * @param buf_capacity    size of out_buf in bytes
 * @param out_len         bytes actually written
 * @param max_ms          hard time limit
 * @param silence_rms     level considered silence
 * @param silence_ms      how long silence must persist to stop early
 */
esp_err_t audio_input_record(void *out_buf, size_t buf_capacity, size_t *out_len,
                             int max_ms, int silence_rms, int silence_ms);

/*
 * Wait until the microphone hears something louder than `trigger_rms`.
 * Used by the energy-VAD wake backend.
 */
esp_err_t audio_input_wait_for_sound(int trigger_rms, int timeout_ms);

/* ------------------------------------------------------------------------ */
/* Audio output - MAX98357A over I2S                                         */
/* ------------------------------------------------------------------------ */

esp_err_t audio_output_init(void);
esp_err_t audio_output_set_enabled(bool enabled);

/*
 * Mute (enable pin low) and idle the TX channel for the radio bring-up
 * window; audio_output_resume() re-enables the channel but leaves the
 * amplifier muted. See radio_bringup.c.
 */
esp_err_t audio_output_stop(void);
esp_err_t audio_output_resume(void);

/*
 * Play raw 16-bit mono PCM. `len` is in bytes.
 * The buffer must stay valid for the duration of the call.
 */
esp_err_t audio_output_play(const void *pcm, size_t len);

/*
 * Streaming playback: begin() un-mutes the amplifier, write() converts a block
 * of 16-bit PCM and pushes it out, end() waits for the DMA to drain and mutes.
 *
 * The point is latency. Buffered playback cannot start until the whole reply
 * has been synthesised; with these three calls the first block can be played
 * while the rest is still arriving.
 */
esp_err_t audio_output_stream_begin(void);
esp_err_t audio_output_stream_write(const void *pcm16, size_t len);
esp_err_t audio_output_stream_end(void);

/* Adapter with the tts_pcm_sink_t signature, so the TTS client can call it. */
esp_err_t audio_output_sink(const void *pcm, size_t len, void *user);

/* ------------------------------------------------------------------------ */
/* iFlytek STT (语音听写)                                                     */
/* ------------------------------------------------------------------------ */

/*
 * Recognise one utterance of 16-bit mono PCM at 16 kHz.
 * On success `out_text` receives a NUL-terminated UTF-8 string that the
 * caller must free().
 */
esp_err_t iflytek_stt_recognize(const void *pcm, size_t len, char **out_text);

/* ------------------------------------------------------------------------ */
/* iFlytek TTS (在线语音合成)                                                  */
/* ------------------------------------------------------------------------ */

/*
 * Synthesise `text` into 16-bit mono PCM at 16 kHz.
 * On success `out_pcm` receives a malloc'd buffer and `out_len` its size.
 * Both must be freed by the caller.
 */
esp_err_t iflytek_tts_synthesize(const char *text, void **out_pcm, size_t *out_len);

/* ------------------------------------------------------------------------ */
/* SiliconFlow STT / TTS                                                     */
/* ------------------------------------------------------------------------ */

/*
 * Same contracts as the two iFlytek functions above, but over plain HTTPS with
 * a single Bearer token instead of a signed WebSocket.
 *
 * Both remain compiled and are reachable directly; ordinary callers should use
 * stt_recognize() / tts_synthesize() below, which forward to whichever backend
 * CONFIG_STT_BACKEND_* / CONFIG_TTS_BACKEND_* selects.
 */
esp_err_t siliconflow_stt_recognize(const void *pcm, size_t len, char **out_text);
esp_err_t siliconflow_tts_synthesize(const char *text, void **out_pcm, size_t *out_len);

/* ------------------------------------------------------------------------ */
/* Backend dispatch                                                          */
/* ------------------------------------------------------------------------ */

/*
 * Recognise one utterance / synthesise one reply using the configured backend.
 *
 * These are what main.c calls. Keeping the choice behind a pair of functions
 * means the turn logic contains no #ifdefs and adding a third provider later
 * touches only the dispatcher.
 */
esp_err_t stt_recognize(const void *pcm, size_t len, char **out_text);
esp_err_t tts_synthesize(const char *text, void **out_pcm, size_t *out_len);

/*
 * Receives one block of raw 16-bit mono PCM as it arrives.
 * Return anything but ESP_OK to abort the transfer.
 */
typedef esp_err_t (*tts_pcm_sink_t)(const void *pcm, size_t len, void *user);

/*
 * Synthesise straight into a sink instead of a buffer. Returns ESP_OK only if
 * the request completed and at least one block was delivered.
 */
esp_err_t tts_stream(const char *text, tts_pcm_sink_t sink, void *user);
esp_err_t siliconflow_tts_stream(const char *text, tts_pcm_sink_t sink, void *user);

/*
 * Endpoint override for the OpenAI-compatible TTS request.
 *
 * The local Qwen3-TTS server speaks exactly the same /v1/audio/speech shape as
 * SiliconFlow, so the same client code serves both - only the URL, the key and
 * the voice name differ. Setting NULL restores the configured SiliconFlow
 * values. Not thread safe; the caller serialises (one turn at a time).
 */
typedef struct {
    const char *base_url;   /* e.g. "http://192.168.1.10:8000/v1" */
    const char *key;        /* "" when the server ignores auth */
    const char *voice;      /* speaker id, e.g. "sohee" */
    const char *model;      /* NULL keeps the configured SiliconFlow model */
} tts_endpoint_t;

void siliconflow_tts_set_endpoint(const tts_endpoint_t *ep);

/*
 * True when the active backend's credentials are still the Kconfig defaults.
 * Used by the boot-time pre-flight check to warn early instead of failing on
 * the first utterance.
 */
bool speech_backend_key_is_placeholder(void);

/* ------------------------------------------------------------------------ */
/* DeepSeek chat                                                             */
/* ------------------------------------------------------------------------ */

/* One conversational turn. */
typedef struct {
    const char *role;    /* "user" or "assistant" */
    const char *content; /* UTF-8 */
} chat_msg_t;

esp_err_t deepseek_client_init(void);

/*
 * Send the conversation so far and return the assistant's reply.
 * On success `out_reply` receives a malloc'd NUL-terminated UTF-8 string.
 */
esp_err_t deepseek_client_chat(const chat_msg_t *history, size_t history_len,
                               char **out_reply);

/* ------------------------------------------------------------------------ */
/* Wake word                                                                 */
/* ------------------------------------------------------------------------ */

esp_err_t wake_word_init(void);

/*
 * Block until the wake word is detected (or, with the VAD backend, until a
 * loud enough sound arrives). `timeout_ms` of 0 means wait forever.
 */
esp_err_t wake_word_wait(int timeout_ms);

/* ------------------------------------------------------------------------ */
/* app_config - thin wrappers over the Kconfig values                        */
/* ------------------------------------------------------------------------ */

const char *app_cfg_wifi_ssid(void);
const char *app_cfg_wifi_password(void);
const char *app_cfg_wifi_ssid2(void);
const char *app_cfg_wifi_password2(void);
const char *app_cfg_deepseek_key(void);
const char *app_cfg_deepseek_base_url(void);
const char *app_cfg_deepseek_model(void);
const char *app_cfg_deepseek_system_prompt(void);
int app_cfg_deepseek_max_tokens(void);
const char *app_cfg_iflytek_app_id(void);
const char *app_cfg_iflytek_api_key(void);
const char *app_cfg_iflytek_api_secret(void);
const char *app_cfg_iflytek_stt_host(void);
const char *app_cfg_iflytek_tts_host(void);
const char *app_cfg_iflytek_tts_voice(void);
int app_cfg_iflytek_tts_speed(void);

const char *app_cfg_siliconflow_base_url(void);
const char *app_cfg_siliconflow_key(void);
const char *app_cfg_siliconflow_stt_model(void);
const char *app_cfg_siliconflow_tts_model(void);
const char *app_cfg_siliconflow_tts_voice(void);
/* speed and gain are JSON numbers carried as strings so the decimal point
 * survives the Kconfig round trip (an int would silently drop it). */
const char *app_cfg_siliconflow_tts_speed(void);
const char *app_cfg_siliconflow_tts_gain(void);

int app_cfg_audio_sample_rate(void);

/*
 * Playback rate, deliberately separate from the capture rate.
 *
 * The amplifier has its own I2S bus since v3, so it is free to run at a
 * different rate from the microphone. A native 24 kHz source (the local Qwen3
 * server) plays without resampling while capture stays at 16 kHz.
 */
int app_cfg_audio_out_sample_rate(void);

/* Local OpenAI-compatible TTS server, tried before the cloud backends. */
bool app_cfg_tts_local_enabled(void);
const char *app_cfg_tts_local_base_url(void);
const char *app_cfg_tts_local_voice(void);
int app_cfg_audio_record_seconds(void);
int app_cfg_audio_silence_ms(void);
int app_cfg_audio_silence_threshold(void);

/* True when every secret still holds its placeholder value. */
bool app_cfg_secrets_are_placeholders(void);

#ifdef __cplusplus
}
#endif
