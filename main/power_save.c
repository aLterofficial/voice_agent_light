/*
 * Power management for running the assistant off a battery bank or a weak
 * USB port.
 *
 * ===========================================================================
 * MEASURED ON THIS BOARD: THIS MODULE HAS NO EFFECT. READ THIS FIRST.
 * ===========================================================================
 *
 * It is kept because the reasoning is worth preserving and because a board
 * with a supply that is only *slightly* short could still benefit, but on this
 * hardware none of it ever runs.
 *
 * Here is why, and it was measured rather than guessed:
 *
 *   The station startup was instrumented step by step, and every reboot
 *   landed in exactly the same place:
 *
 *       [4/6] esp_wifi_init (RF calibration)     <-- completes
 *       [5/6] esp_wifi_init returned             <-- so calibration survives
 *       [6/6] esp_wifi_start                     <-- never returns
 *
 *   Two things follow, and both are bad news for a firmware-only fix:
 *
 *   1. The current spike that collapses the rail is inside esp_wifi_start(),
 *      NOT inside RF calibration. The earlier belief that calibration was the
 *      culprit (recorded in BROWNOUT-DIAGNOSIS.md) is wrong.
 *
 *   2. esp_wifi_set_max_tx_power() is documented to require esp_wifi_start()
 *      to have already run - it returns ESP_ERR_WIFI_NOT_STARTED before that.
 *      Since the reset happens *inside* esp_wifi_start(), the call is never
 *      reached. power_save_init() below has therefore never executed even
 *      once on this board. The boot log confirms it: startup stops at the I2S
 *      lines and the "LOW POWER MODE ENABLED" banner never appears.
 *
 *   CONFIG_ESP_PHY_MAX_TX_POWER cannot rescue it either. That option is a
 *   hidden internal one (declared as "config ... int" with a bare "default"),
 *   so Kconfig ignores any value written into sdkconfig and re-derives it
 *   from ESP_PHY_MAX_WIFI_TX_POWER, whose floor is 10 dBm.
 *
 * The conclusion is that the supply is short enough that no software lever
 * reaches it. See the brownout section of README.md for what to do instead:
 * a supply that holds the 3V3 rail up under the radio's burst, not another
 * setting.
 *
 * ---------------------------------------------------------------------------
 * What the module does, for the record
 * ---------------------------------------------------------------------------
 *
 * Three knobs exist for TX power:
 *
 *   CONFIG_ESP_PHY_MAX_WIFI_TX_POWER     Kconfig, floor of 10 dBm - the value
 *                                        the driver defaults to.
 *
 *   CONFIG_ESP_PHY_MAX_TX_POWER          Kconfig, no floor, but hidden from
 *                                        menuconfig and re-derived (above).
 *                                        It feeds PHY_TX_POWER_LOWEST for the
 *                                        calibration step, and only when the
 *                                        previous reset was a brownout
 *                                        (phy_init.c checks the reset reason).
 *
 *   esp_wifi_set_max_tx_power()          Runtime API, 8..84 in units of
 *                                        0.25 dBm = 2..20 dBm. This is the
 *                                        only lever with real headroom, and it
 *                                        is unreachable on this board.
 *
 * The trade-off would be range: 2 dBm is about 1.6 mW, fine for a phone
 * hotspot on the same table, useless across a room.
 */

#include <inttypes.h>

#include "app_internal.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if CONFIG_LOW_POWER_MODE
static const char *TAG = "power";

#if CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

/*
 * Apply the runtime TX power cap.
 *
 * Called after esp_wifi_start(), because the API rejects calls made before
 * the interface is up with ESP_ERR_WIFI_NOT_STARTED.
 *
 * Note the driver's own default comes from CONFIG_ESP_PHY_MAX_WIFI_TX_POWER
 * (10 dBm), so lowering it here is a genuine reduction and not merely
 * restating the Kconfig value.
 */
static void apply_tx_power_limit(void)
{
    const int8_t want = CONFIG_LOW_POWER_TX_POWER_DBM * 4; /* API unit: 0.25 dBm */

    int8_t before = 0;
    if (esp_wifi_get_max_tx_power(&before) == ESP_OK) {
        ESP_LOGI(TAG, "TX power before: %d (%.2f dBm)", before, before * 0.25);
    }

    esp_err_t err = esp_wifi_set_max_tx_power(want);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cannot set TX power (%s) - continuing at the default",
                 esp_err_to_name(err));
        return;
    }

    int8_t after = 0;
    if (esp_wifi_get_max_tx_power(&after) == ESP_OK) {
        ESP_LOGW(TAG, "TX power capped at %d (%.2f dBm) - range is reduced, "
                      "keep the hotspot close",
                 after, after * 0.25);
    }
}

/*
 * Ask the radio to sleep between beacons.
 *
 * WIFI_PS_MIN_MODEM wakes on every DTIM; MAX_MODEM uses listen_interval and
 * therefore sleeps longer. The cost of MAX_MODEM is latency and, on a busy
 * or lossy AP, missed beacons - the driver can decide the AP has gone away,
 * which shows up as a disassociation. For an assistant that talks in bursts
 * of a few hundred ms, adding tens of ms of latency is acceptable; losing
 * the link is not, so the default here is MIN_MODEM and MAX_MODEM is opt-in.
 *
 * The main firmware sets WIFI_PS_NONE for audio smoothness (see wifi_mgr.c).
 * That choice is deliberately overridden here: with NONE the receiver stays
 * biased continuously, which is a large constant draw on a supply that
 * cannot even survive the transmit bursts.
 */
static void apply_wifi_sleep(void)
{
#if CONFIG_LOW_POWER_WIFI_MAX_MODEM
    esp_err_t err = esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
    ESP_LOGW(TAG, "WiFi power save: MAX_MODEM (longest sleep, most latency)");
#else
    esp_err_t err = esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    ESP_LOGI(TAG, "WiFi power save: MIN_MODEM (wakes every DTIM)");
#endif
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cannot enable WiFi power save: %s", esp_err_to_name(err));
    }
}

/*
 * Dynamic frequency scaling.
 *
 * Lowering the CPU ceiling cuts the compute share of the current budget. It
 * does not touch the radio burst, which is what actually trips the brownout,
 * so on its own it has already been measured as ineffective (see
 * BROWNOUT-DIAGNOSIS.md). It is included because it stacks with the TX power
 * reduction rather than competing with it.
 *
 * Requires CONFIG_PM_ENABLE. Without it esp_pm_configure() is not linked in.
 */
static void apply_dfs(void)
{
#if CONFIG_PM_ENABLE
    esp_pm_config_t pm = {
        .max_freq_mhz = CONFIG_LOW_POWER_CPU_MAX_MHZ,
        .min_freq_mhz = CONFIG_LOW_POWER_CPU_MIN_MHZ,
        /* Light sleep needs a stable wake source and complicates the I2S
         * clocking, and the assistant spends most of its time with the
         * microphone running, so keep it off. */
        .light_sleep_enable = false,
    };

    esp_err_t err = esp_pm_configure(&pm);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cannot configure power management: %s (is CONFIG_PM_ENABLE set?)",
                 esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "DFS: CPU %d..%d MHz", pm.min_freq_mhz, pm.max_freq_mhz);
#else
    ESP_LOGI(TAG, "DFS not enabled (set CONFIG_PM_ENABLE to use it)");
#endif
}

esp_err_t power_save_init(void)
{
    ESP_LOGW(TAG, "");
    ESP_LOGW(TAG, "*** LOW POWER MODE ENABLED ***");
    ESP_LOGW(TAG, "This reduces the radio's peak current so the board may");
    ESP_LOGW(TAG, "survive on a battery bank or a weak USB port. It does NOT");
    ESP_LOGW(TAG, "fix the supply - see BROWNOUT-DIAGNOSIS.md.");
    ESP_LOGW(TAG, "");

    apply_dfs();
    apply_wifi_sleep();
    apply_tx_power_limit();

    return ESP_OK;
}

/*
 * Called BEFORE the radio is brought up.
 *
 * Kept as the place for anything that has to happen before esp_wifi_start(),
 * and as a boot-log record of the compile-time PHY setting, but it is now
 * known that nothing set here changes the outcome on this board.
 *
 * The original reasoning was that the dangerous moment is RF calibration
 * inside esp_wifi_init(), and that the runtime TX power API is only legal
 * after esp_wifi_start(), leaving calibration unprotected. Measuring where
 * the resets actually land disproved the premise:
 *
 *     [4/6] esp_wifi_init (RF calibration)    <-- completes
 *     [5/6] esp_wifi_init returned            <-- calibration survives
 *     [6/6] esp_wifi_start                    <-- reset lands here, forever
 *
 * So the spike is in esp_wifi_start(), and the runtime API is unreachable for
 * a different reason than assumed: the function that would call it never
 * returns. See the file header.
 *
 * A note on CONFIG_ESP_PHY_MAX_TX_POWER, since it looks like the obvious
 * pre-start lever and is not: it is declared as a hidden internal option
 * ("config ESP_PHY_MAX_TX_POWER" with no prompt, only a default), so Kconfig
 * discards any value assigned to it in sdkconfig and recomputes it from
 * ESP_PHY_MAX_WIFI_TX_POWER. Writing 4 there has no effect; the build reports
 * 10. The PHY_TX_POWER_LOWEST it feeds is therefore pinned at 40/52, and the
 * only way to move it is to lower ESP_PHY_MAX_WIFI_TX_POWER, whose floor is
 * 10 dBm to begin with.
 */
esp_err_t power_save_init_before_radio(void)
{
    ESP_LOGW(TAG, "low power mode: pre-radio phase");
    ESP_LOGI(TAG, "  PHY calibration TX power derives from "
                  "CONFIG_ESP_PHY_MAX_TX_POWER=%d (hidden option, not "
                  "overridable from sdkconfig)",
             CONFIG_ESP_PHY_MAX_TX_POWER);

    /*
     * NOTE: esp_wifi_set_max_tx_power() cannot be called yet - it needs
     * esp_wifi_start() to have run, and returns ESP_ERR_WIFI_NOT_STARTED
     * before that. The runtime cap is applied in power_save_init(), which on
     * this board is never reached.
     */
    return ESP_OK;
}

/*
 * Called after a reconnect, because esp_wifi_stop()/start() resets the TX
 * power to the PHY init default. The reconnect path in wifi_mgr.c does not
 * currently stop the interface, but re-applying is cheap and keeps this
 * correct if that changes.
 */
void power_save_reapply(void)
{
    apply_tx_power_limit();
    apply_wifi_sleep();
}

#else /* !CONFIG_LOW_POWER_MODE */

esp_err_t power_save_init(void)
{
    return ESP_OK;
}

esp_err_t power_save_init_before_radio(void)
{
    return ESP_OK;
}

void power_save_reapply(void)
{
}

#endif
