/*
 * Radio bring-up: current shedding for the moment the rail collapses.
 *
 * WHY THIS EXISTS
 * ---------------
 * Instrumented measurements (2026-10-02, board on USB, COM5) put the reset
 * inside esp_wifi_init(), not inside esp_wifi_start() as the earlier notes
 * claimed. The last thing the boot prints before the reset is the WiFi
 * library's own "Allocate MAC register memory", and the highest application
 * timestamp ever reached in a 20 s window is 762 ms - esp_wifi_start() is
 * never reached at all on this supply.
 *
 * That single fact explains why every previous attempt failed:
 *
 *   * esp_wifi_set_max_tx_power() needs esp_wifi_start() to have returned,
 *     so it can never run;
 *   * the step markers in wifi_mgr.c only bracket whole calls, so the
 *     resolution was too coarse to see that the failure moved;
 *   * "low power mode has no effect" was measured against a reset point that
 *     is upstream of everything the mode changes.
 *
 * WHAT IT DOES
 * ------------
 * Bring-up is split into three phases, and each is individually switchable:
 *
 *   prepare()  - before the radio: cap the CPU at 80 MHz (the S3 boots at
 *                240 MHz and reaches esp_wifi_init() at full clock), stop
 *                both I2S channels, blank the LED. Everything that switches
 *                at the same instant as the MAC/PHY power-up is switched off
 *                first, so the radio burst sees the smallest possible
 *                baseline load.
 *   log()      - optional per-step marker with elapsed time and free
 *                internal heap, at a resolution fine enough to see a
 *                millisecond-scale improvement inside esp_wifi_init().
 *   finish()   - restore the LED, both I2S channels and the CPU ceiling.
 *
 * WHAT IT CANNOT DO
 * -----------------
 * It cannot raise the voltage. The measured rail dip is about 0.86 V on a
 * 3.3 V rail, which is a supply problem, not a firmware one. The honest
 * expectation is that these changes shave tens of milliamps off a load the
 * supply cannot carry, so they may well not fix anything - but they cost one
 * build and no soldering, and the measurement either way is worth having.
 *
 * See VERIFY-2026-10-02.md and BROWNOUT-DIAGNOSIS.md.
 */

#include <inttypes.h>

#include "app_internal.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if defined(CONFIG_BRINGUP_TRACE) || defined(CONFIG_BROWNOUT_DIAG_TRACE)
#define RADIO_TRACE 1
#else
#define RADIO_TRACE 0
#endif

#if CONFIG_BRINGUP_LOW_CPU && CONFIG_PM_ENABLE
#include "esp_pm.h"
static esp_pm_lock_handle_t s_cpu_lock = NULL;
#endif

static const char *TAG = "bringup";
static int64_t s_t0 = 0;

/* ------------------------------------------------------------------------ */
/* Tracing                                                                   */
/* ------------------------------------------------------------------------ */

/*
 * Markers are ESP_LOGW so they survive any configured log level: the whole
 * point is to have the last line survive the reset, and the log level is one
 * more thing that could be misconfigured under pressure.
 */
void radio_bringup_mark(const char *what)
{
#if RADIO_TRACE
    const int64_t now = esp_timer_get_time();
    ESP_LOGW(TAG, "[%6" PRId64 " ms] %s  (int heap %u KB)",
             (now - s_t0) / 1000, what,
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
#else
    (void)what;
#endif
}

/* ------------------------------------------------------------------------ */
/* Load shedding                                                             */
/* ------------------------------------------------------------------------ */

#if CONFIG_BRINGUP_QUIET_PERIPHERALS

void radio_bringup_prepare(void)
{
    if (s_t0 == 0) {
        s_t0 = esp_timer_get_time();
    }

    /* Order matters: the marker is logged AFTER the shedding, not before.
     * Logging it before was measurably unreliable - the UART write landed in
     * the same window as the I2S shutdown and came out as line noise on every
     * boot (observed 2026-10-02). A marker that is itself unreliable is worse
     * than no marker, so the string is only emitted once the line is quiet. */

    /* The LED first: the RGB strip is the only thing here that can draw tens
     * of milliamps, and a heartbeat pattern is a switching load. */
    led_status_quiet();

    /* Then the audio clocks. Both must go quiet before the radio powers up -
     * see the note in audio_output_stop(). Errors are logged, not fatal: a
     * failure here means a slightly higher baseline, not a broken boot. */
    esp_err_t err_out = audio_output_stop();
    if (err_out != ESP_OK) {
        ESP_LOGW(TAG, "could not stop I2S TX: %s", esp_err_to_name(err_out));
    }
    esp_err_t err_in = audio_input_stop();
    if (err_in != ESP_OK) {
        ESP_LOGW(TAG, "could not stop I2S RX: %s", esp_err_to_name(err_in));
    }

    /* Give the amplifier's supply a moment to settle before the radio pulls
     * its own inrush current. The delay also lets the console settle: logging
     * this marker immediately after the I2S shutdown produced a truncated
     * line on every boot (observed 2026-10-02), which is precisely the kind of
     * unreliable instrumentation that makes a diagnosis untrustworthy. */
    vTaskDelay(pdMS_TO_TICKS(20));

    radio_bringup_mark("prepare: LED blanked, both I2S channels idled");
}

void radio_bringup_finish(void)
{
    radio_bringup_mark("finish: restoring I2S and LED");

    esp_err_t err_in = audio_input_resume();
    if (err_in != ESP_OK) {
        ESP_LOGE(TAG, "I2S RX did not come back: %s - recording is broken, "
                      "reflash with CONFIG_BRINGUP_QUIET_PERIPHERALS=n",
                 esp_err_to_name(err_in));
    }
    esp_err_t err_out = audio_output_resume();
    if (err_out != ESP_OK) {
        ESP_LOGE(TAG, "I2S TX did not come back: %s - playback is broken, "
                      "reflash with CONFIG_BRINGUP_QUIET_PERIPHERALS=n",
                 esp_err_to_name(err_out));
    }

    led_status_unquiet();
}

#else

void radio_bringup_prepare(void)
{
    if (s_t0 == 0) {
        s_t0 = esp_timer_get_time();
    }
}

void radio_bringup_finish(void)
{
}

#endif /* CONFIG_BRINGUP_QUIET_PERIPHERALS */

/* ------------------------------------------------------------------------ */
/* CPU ceiling                                                               */
/* ------------------------------------------------------------------------ */

/*
 * acquire_cpu_ceiling()/release_cpu_ceiling() bound the CPU for the radio's
 * power-up window only.
 *
 * A MAX_FREQ lock is used rather than esp_pm_configure() because it is
 * precise: while the lock is held the governor cannot clock above 80 MHz, and
 * releasing it restores whatever ceiling the application configured (240 MHz
 * by default, or CONFIG_LOW_POWER_CPU_MAX_MHZ in low power mode).
 *
 * The call is a no-op if PM is not compiled in, which is why the switch is
 * guarded rather than failing: on a board without CONFIG_PM_ENABLE the other
 * parts of this module still apply.
 */
void radio_bringup_cpu_before_radio(void)
{
#if CONFIG_BRINGUP_LOW_CPU
#if CONFIG_PM_ENABLE
    if (s_cpu_lock == NULL) {
        esp_err_t err = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "radio_bu",
                                           &s_cpu_lock);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "cannot create PM lock (%s) - CPU stays at %d MHz",
                     esp_err_to_name(err), CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
            return;
        }
    }
    esp_err_t err = esp_pm_lock_acquire(s_cpu_lock);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cannot acquire PM lock: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGW(TAG, "CPU capped at 80 MHz for radio bring-up "
                  "(was free to run at %d MHz)", CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
#else
    ESP_LOGW(TAG, "CONFIG_BRINGUP_LOW_CPU is set but CONFIG_PM_ENABLE is not - "
                  "the CPU ceiling cannot be changed");
#endif
#else
    radio_bringup_mark("CPU ceiling unchanged (CONFIG_BRINGUP_LOW_CPU=n)");
#endif
}

void radio_bringup_cpu_after_radio(void)
{
#if CONFIG_BRINGUP_LOW_CPU && CONFIG_PM_ENABLE
    if (s_cpu_lock != NULL) {
        esp_err_t err = esp_pm_lock_release(s_cpu_lock);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "cannot release PM lock: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "CPU ceiling restored (radio is up)");
        }
    }
#endif
}
