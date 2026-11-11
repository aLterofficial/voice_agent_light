/*
 * WiFi station management - multi-network with automatic fallback.
 *
 * Why this exists in this shape:
 *
 *   A competition or demo venue rarely has the same network as the lab. The
 *   previous version of this file stored exactly one SSID and gave up after
 *   CONFIG_WIFI_MAX_RETRY attempts, which meant:
 *
 *     * moving between the lab WiFi and a phone hotspot required a recompile
 *       and a reflash, and
 *     * if the hotspot was not up yet at boot, the device latched into the
 *       error state and never tried again - a power cycle was the only cure.
 *
 *   This version keeps a small list of candidate networks (any of them may be
 *   a phone hotspot), walks the list on every attempt, and retries forever
 *   with exponential backoff. A hotspot that appears late is picked up
 *   without intervention.
 *
 * The ESP32-S3 radio is 2.4 GHz only. A phone hotspot in its default
 * "5 GHz" mode is invisible to it, which is the single most common reason a
 * hotspot connection fails. Use the 'w' console command (console_cmd.c) to
 * list what the radio can actually see.
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_internal.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *TAG = "wifi";

/*
 * Startup step markers, for locating a brownout.
 *
 * The radio's startup is the tallest current spike the board sees, and on a
 * marginal supply the last step a boot manages to print is the answer to
 * where it died. Instrumenting the station startup is what established that
 * this board completes esp_wifi_init() (and its RF calibration) and resets
 * inside esp_wifi_start() - see the header of main/power_save.c.
 *
 * The markers use ESP_LOGW deliberately, so they appear regardless of the
 * configured log level, and they are compiled out unless
 * CONFIG_BROWNOUT_DIAG_TRACE is set, because on a healthy board they are pure
 * noise in every boot.
 *
 * Each call is logged before and after, so the pair brackets the call and the
 * last line printed identifies the call that did not return.
 */
#if CONFIG_BROWNOUT_DIAG_TRACE
#define STEP_BEGIN(step, what)  ESP_LOGW(TAG, "[" step "] " what)
#define STEP_END(step, what)    ESP_LOGW(TAG, "[" step "] " what " returned")
#else
#define STEP_BEGIN(step, what)  do { } while (0)
#define STEP_END(step, what)    do { } while (0)
#endif

#define BIT_CONNECTED BIT0

/* Backoff between full passes over the candidate list. */
#define RETRY_MIN_MS   1000
#define RETRY_MAX_MS  30000

static EventGroupHandle_t s_events;
static volatile bool s_connected = false;

/* Set by wifi_mgr_reconnect_now() to make the next retry immediate. */
static volatile bool s_retry_now = false;

/* ------------------------------------------------------------------------ */
/* Candidate list                                                            */
/* ------------------------------------------------------------------------ */

/*
 * Both Kconfig slots are optional. Slot 1 (CONFIG_WIFI_SSID) is the primary -
 * the lab/venue network. Slot 2 (CONFIG_WIFI_SSID_2) is meant for a phone
 * hotspot and defaults to a sentinel that is skipped when left alone.
 *
 * Returns the number of usable entries (0, 1 or 2).
 */
static int collect_candidates(const char **ssids, const char **passes, int max)
{
    int n = 0;

    const char *ssid1 = app_cfg_wifi_ssid();
    if (ssid1 != NULL && ssid1[0] != '\0' && strcmp(ssid1, "YOUR_WIFI_SSID") != 0) {
        ssids[n] = ssid1;
        passes[n] = app_cfg_wifi_password();
        n++;
    }

    if (n < max) {
        const char *ssid2 = app_cfg_wifi_ssid2();
        if (ssid2 != NULL && ssid2[0] != '\0' && strcmp(ssid2, "YOUR_WIFI_SSID_2") != 0) {
            ssids[n] = ssid2;
            passes[n] = app_cfg_wifi_password2();
            n++;
        }
    }

    return n;
}

/* ------------------------------------------------------------------------ */
/* Disconnect reason -> human readable                                       */
/* ------------------------------------------------------------------------ */

/*
 * The reason codes worth naming explicitly. Everything else is printed
 * numerically, which is still enough to look up in the IDF headers.
 *
 * The two that matter most in practice:
 *
 *   201 NO_AP_FOUND        almost always a 5 GHz hotspot, or the SSID is
 *                          misspelled, or the AP is out of range
 *   15  HANDSHAKE_TIMEOUT  wrong password (or WPA3 on an AP the station
 *                          cannot negotiate with)
 */
static const char *reason_text(uint8_t reason)
{
    switch (reason) {
    case WIFI_REASON_NO_AP_FOUND:              return "no AP found - is the hotspot 2.4 GHz? is the SSID right?";
    case WIFI_REASON_AUTH_FAIL:                return "authentication failed - wrong password";
    case WIFI_REASON_ASSOC_FAIL:               return "association failed";
    case WIFI_REASON_HANDSHAKE_TIMEOUT:        return "4-way handshake timeout - wrong password?";
    case WIFI_REASON_CONNECTION_FAIL:          return "connection failed";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:   return "4-way handshake timeout - wrong password?";
    case WIFI_REASON_BEACON_TIMEOUT:           return "beacon timeout - AP went away";
    case WIFI_REASON_AUTH_EXPIRE:              return "authentication expired";
    case WIFI_REASON_ASSOC_LEAVE:              return "deassociated - STA leaving";
    case WIFI_REASON_ASSOC_TOOMANY:            return "AP is full - too many stations";
    case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY: return "AP found but security is incompatible";
    case WIFI_REASON_DISASSOC_DUE_TO_INACTIVITY: return "disassociated - AP idle timeout";
    case WIFI_REASON_CIPHER_SUITE_REJECTED:    return "cipher suite rejected - security mode mismatch";
    default:                                   return NULL;
    }
}

static void log_disconnect_reason(uint8_t reason)
{
    const char *text = reason_text(reason);
    if (text != NULL) {
        ESP_LOGW(TAG, "disconnected: reason %u = %s", reason, text);
    } else {
        ESP_LOGW(TAG, "disconnected: reason %u", reason);
    }

    /* The two reasons that point at a configuration mistake rather than a
     * transient problem - worth an explicit hint because they are what a
     * venue failure almost always turns out to be. */
    if (reason == WIFI_REASON_NO_AP_FOUND) {
        ESP_LOGW(TAG, "  hint: the ESP32-S3 has a 2.4 GHz radio only. On most");
        ESP_LOGW(TAG, "  phones the hotspot defaults to 5 GHz - enable");
        ESP_LOGW(TAG, "  \"maximise compatibility\" / set the AP band to 2.4 GHz.");
        ESP_LOGW(TAG, "  run 'w' in the console to list what the radio can see.");
    }
}

/* ------------------------------------------------------------------------ */
/* Connection state machine                                                  */
/* ------------------------------------------------------------------------ */

static void start_attempt(void)
{
    const char *ssids[CONFIG_WIFI_CANDIDATE_MAX];
    const char *passes[CONFIG_WIFI_CANDIDATE_MAX];
    const int n = collect_candidates(ssids, passes, CONFIG_WIFI_CANDIDATE_MAX);

    if (n == 0) {
        ESP_LOGE(TAG, "no candidate networks configured - set the SSID(s)");
        led_status_set(LED_ERROR);
        return;
    }

    static int cursor = 0;
    cursor %= n;

    wifi_config_t wc = {0};
    strlcpy((char *)wc.sta.ssid, ssids[cursor], sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, passes[cursor], sizeof(wc.sta.password));

    /*
     * Authentication and scanning settings, chosen for a hotspot that may be
     * WPA2, WPA3 or "WPA2/WPA3 mixed":
     *
     *   threshold.authmode WPA2_PSK
     *       Accept WPA2 and anything stronger. Setting OPEN here would also
     *       work but lets the station associate with an open rogue AP.
     *
     *   scan_method ALL_CHANNEL_SCAN + sort by signal
     *       FASTA scan stops at the first match by channel order; a full scan
     *       picks the strongest BSSID, which matters when a repeater and the
     *       hotspot share an SSID.
     *
     *   pmf capable but not required
     *       WPA3 mandates PMF; WPA2 APs tolerate it being advertised. This is
     *       the combination that connects to both.
     *
     *   sae_pwe_h2e BOTH
     *       Some WPA3 APs only do hunt-and-peck, others only H2E. BOTH lets
     *       the station try whichever the AP offers, which is why it is the
     *       IDF default for this field.
     */
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wc.sta.scan_method        = WIFI_ALL_CHANNEL_SCAN;
    wc.sta.sort_method        = WIFI_CONNECT_AP_BY_SIGNAL;
    wc.sta.pmf_cfg.capable    = true;
    wc.sta.pmf_cfg.required   = false;
    wc.sta.sae_pwe_h2e        = WPA3_SAE_PWE_BOTH;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));

    ESP_LOGI(TAG, "connecting to \"%s\" (candidate %d/%d) ...",
             ssids[cursor], cursor + 1, n);
    esp_wifi_connect();

    cursor++; /* next attempt starts with the following candidate */
}

/*
 * Backoff lives in a dedicated task rather than in the event handler: the
 * event handler runs on the WiFi task, and calling esp_wifi_connect() from
 * inside it after a delay would block the driver.
 */
static void reconnect_task(void *arg)
{
    uint32_t delay_ms = RETRY_MIN_MS;

    while (1) {
        /* Wait until a disconnect has actually been signalled. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (s_connected) {
            continue;
        }

        /* A manual reconnect request skips the wait entirely. */
        if (s_retry_now) {
            s_retry_now = false;
            delay_ms = RETRY_MIN_MS;
        } else {
            ESP_LOGW(TAG, "retrying in %" PRIu32 " ms", delay_ms);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }

        if (!s_connected) {
            start_attempt();
        }

        /* Exponential backoff, capped. A successful connection resets it. */
        delay_ms *= 2;
        if (delay_ms > RETRY_MAX_MS) {
            delay_ms = RETRY_MAX_MS;
        }
    }
}

static TaskHandle_t s_reconnect_task = NULL;

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        start_attempt();

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t *)data;
        const bool was_connected = s_connected;
        s_connected = false;

        if (was_connected) {
            ESP_LOGW(TAG, "lost the connection");
        }
        log_disconnect_reason(d->reason);
        led_status_set(LED_WIFI);

        if (s_reconnect_task != NULL) {
            xTaskNotifyGive(s_reconnect_task);
        }

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        wifi_ap_record_t ap = {0};
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            ESP_LOGI(TAG, "connected to \"%s\" channel %u rssi %d dBm",
                     ap.ssid, ap.primary, ap.rssi);
        }
        ESP_LOGI(TAG, "got IP " IPSTR, IP2STR(&e->ip_info.ip));
        s_connected = true;
        xEventGroupSetBits(s_events, BIT_CONNECTED);
    }
}

/* ------------------------------------------------------------------------ */
/* Public API                                                                */
/* ------------------------------------------------------------------------ */

esp_err_t wifi_mgr_start(void)
{
    /* See STEP_BEGIN above: these markers locate a brownout.
     *
     * MEASURED 2026-10-02: the reset lands inside esp_wifi_init(), not in
     * esp_wifi_start(). The boot log stops right after the WiFi library's
     * "Allocate MAC register memory" and esp_wifi_start() is never reached,
     * so every runtime TX-power API is unreachable by construction. The
     * comment here previously claimed the opposite; it was inferred from
     * markers that were too coarse to tell the two apart. */
    STEP_BEGIN("1/5", "nvs_flash_init");

    /* --- NVS is required by the WiFi driver --- */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    STEP_END("1/5", "nvs_flash_init");

    s_events = xEventGroupCreate();
    if (s_events == NULL) {
        return ESP_ERR_NO_MEM;
    }

    STEP_BEGIN("2/5", "esp_netif_init");
    ESP_ERROR_CHECK(esp_netif_init());
    STEP_END("2/5", "esp_netif_init");

    STEP_BEGIN("3/5", "esp_event_loop_create_default");
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    radio_bringup_mark("netif/event loop ready");
    STEP_END("3/5", "esp_event_loop_create_default");

    STEP_BEGIN("4/5", "esp_wifi_init (RF calibration)");
    /* A second, finer layer of markers inside the call that actually dies.
     * The STEP_ pair only brackets the whole call, which is far too coarse:
     * the reset lands ~100 ms into this window, in the WiFi library's own
     * MAC/PHY power-up. mark() records elapsed time and free internal heap so
     * a change that shifts the death by a few milliseconds is still visible. */
    radio_bringup_mark("esp_wifi_init: entering (MAC/PHY power-up inside)");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t init_err = esp_wifi_init(&cfg);
    radio_bringup_mark("esp_wifi_init: returned");
    ESP_ERROR_CHECK(init_err);
    STEP_END("4/5", "esp_wifi_init");

    /*
     * Keep the station config in RAM only. The default (FLASH) makes the
     * driver write to NVS on every set_config, which is wasted flash wear
     * here: the credentials come from Kconfig and are applied at every boot.
     */
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &on_wifi_event, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    radio_bringup_mark("sta mode set, about to call esp_wifi_start()");

    STEP_BEGIN("5/5", "esp_wifi_start (first transmit)");
    ESP_ERROR_CHECK(esp_wifi_start());
    STEP_END("5/5", "esp_wifi_start");
    radio_bringup_mark("esp_wifi_start returned - radio is up");

    /* Audio streaming wants the radio awake; power save makes the TLS round
     * trips noticeably slower and jittery. It also costs current, which is a
     * known problem on this board - see BROWNOUT-DIAGNOSIS.md.
     *
     * Note that power_save_init() may override this to MIN_MODEM, so the
     * setting here only applies when low power mode is off. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    {
        wifi_ps_type_t ps = WIFI_PS_NONE;
        esp_wifi_get_ps(&ps);
        ESP_LOGI(TAG, "wifi power save in force: %d (0=NONE 1=MIN_MODEM 2=MAX_MODEM)",
                 (int)ps);
    }

    if (xTaskCreate(reconnect_task, "wifi_reconn", 3072, NULL, 4, &s_reconnect_task)
            != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

bool wifi_mgr_is_connected(void)
{
    return s_connected;
}

void wifi_mgr_wait_connected(void)
{
    /*
     * Wait for the first successful connection. There is deliberately no
     * failure exit: the reconnect task retries forever, so the caller would
     * rather block here than have the assistant start up with no network.
     */
    ESP_LOGI(TAG, "waiting for a network ...");
    xEventGroupWaitBits(s_events, BIT_CONNECTED, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG, "network up");
}

/* ------------------------------------------------------------------------ */
/* Diagnostics                                                               */
/* ------------------------------------------------------------------------ */

/*
 * Fill `out` with the SSIDs this manager would actually try, NULL-terminated,
 * so callers can show the user the same list the connection logic uses
 * without duplicating the sentinel checks.
 */
void wifi_mgr_describe_candidates(const char **out, int max)
{
    if (out == NULL || max <= 0) {
        return;
    }
    for (int i = 0; i < max; i++) {
        out[i] = NULL;
    }

    const char *ssids[CONFIG_WIFI_CANDIDATE_MAX];
    const char *passes[CONFIG_WIFI_CANDIDATE_MAX];
    const int n = collect_candidates(ssids, passes, CONFIG_WIFI_CANDIDATE_MAX);

    for (int i = 0; i < n && i < max; i++) {
        out[i] = ssids[i];
    }
}

/*
 * Drop the current link (if any) and run a connection attempt straight away,
 * resetting the backoff. Used by the console 'r' command so a user standing
 * at the venue does not have to wait out a 30 s backoff after switching the
 * hotspot on.
 */
void wifi_mgr_reconnect_now(void)
{
    s_retry_now = true;

    if (s_connected) {
        esp_wifi_disconnect();
    } else if (s_reconnect_task != NULL) {
        /* Not connected and probably waiting out a backoff: cut it short. */
        xTaskNotifyGive(s_reconnect_task);
    }
}

void wifi_mgr_scan_and_report(void)
{
    /*
     * Blocking scan. Must not be called while a connection attempt is in
     * flight - IDF returns ESP_ERR_WIFI_STATE then. In practice this is run
     * from the console while the device is stuck failing to connect, which is
     * exactly when the radio is idle.
     */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "scanning for access points ...");

    wifi_scan_config_t scan = {
        .ssid        = NULL,
        .bssid       = NULL,
        .channel     = 0,
        .show_hidden = true,
        .scan_type   = WIFI_SCAN_TYPE_ACTIVE,
    };

    esp_err_t err = esp_wifi_scan_start(&scan, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "scan failed: %s", esp_err_to_name(err));
        if (err == ESP_ERR_WIFI_STATE) {
            ESP_LOGE(TAG, "  (a connection attempt is in progress - try again "
                          "in a second)");
        }
        return;
    }

    uint16_t found = 0;
    esp_wifi_scan_get_ap_num(&found);
    if (found == 0) {
        ESP_LOGW(TAG, "no access points visible at all");
        ESP_LOGW(TAG, "  -> that points at antenna/RF hardware, or a supply "
                      "too weak to run the receiver");
        esp_wifi_clear_ap_list();
        return;
    }

    /* Cap the report so a crowded venue does not need a huge allocation. */
    uint16_t want = found > 25 ? 25 : found;
    wifi_ap_record_t *recs = calloc(want, sizeof(*recs));
    if (recs == NULL) {
        ESP_LOGE(TAG, "out of memory for the scan results");
        esp_wifi_clear_ap_list();
        return;
    }

    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&want, recs));

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  %-32s %4s %4s  %s", "SSID", "chan", "rssi", "security");
    ESP_LOGI(TAG, "  %-32s %4s %4s  %s", "----", "----", "----", "--------");

    const char *configured[CONFIG_WIFI_CANDIDATE_MAX];
    const char *dummy_passes[CONFIG_WIFI_CANDIDATE_MAX];
    const int n_cfg = collect_candidates(configured, dummy_passes,
                                         CONFIG_WIFI_CANDIDATE_MAX);

    for (uint16_t i = 0; i < want; i++) {
        const wifi_ap_record_t *r = &recs[i];

        const char *sec = "?";
        switch (r->authmode) {
        case WIFI_AUTH_OPEN:            sec = "open";        break;
        case WIFI_AUTH_WEP:             sec = "WEP";         break;
        case WIFI_AUTH_WPA_PSK:         sec = "WPA";         break;
        case WIFI_AUTH_WPA2_PSK:        sec = "WPA2";        break;
        case WIFI_AUTH_WPA_WPA2_PSK:    sec = "WPA/WPA2";    break;
        case WIFI_AUTH_WPA3_PSK:        sec = "WPA3";        break;
        case WIFI_AUTH_WPA2_WPA3_PSK:   sec = "WPA2/WPA3";   break;
        case WIFI_AUTH_WPA2_ENTERPRISE: sec = "WPA2-ENT";    break;
        default:                        sec = "other";       break;
        }

        /* Mark the networks this device is configured to use. */
        bool is_configured = false;
        for (int c = 0; c < n_cfg; c++) {
            if (strcmp((const char *)r->ssid, configured[c]) == 0) {
                is_configured = true;
                break;
            }
        }

        ESP_LOGI(TAG, "  %-32s %4u %4d  %s%s",
                 r->ssid, r->primary, r->rssi, sec,
                 is_configured ? "   <- configured" : "");
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  %u AP(s) total, %u shown", found, want);
    ESP_LOGI(TAG, "  note: this list is 2.4 GHz only. If your hotspot is");
    ESP_LOGI(TAG, "  missing here, it is broadcasting on 5 GHz - switch it to");
    ESP_LOGI(TAG, "  2.4 GHz (or enable maximum compatibility) on the phone.");
    ESP_LOGI(TAG, "");

    if (n_cfg == 0) {
        ESP_LOGW(TAG, "  no SSID is configured yet - run configure.ps1 or "
                      "idf.py menuconfig");
    } else {
        for (int c = 0; c < n_cfg; c++) {
            bool seen = false;
            for (uint16_t i = 0; i < want && !seen; i++) {
                if (strcmp((const char *)recs[i].ssid, configured[c]) == 0) {
                    seen = true;
                }
            }
            ESP_LOGI(TAG, "  configured \"%s\": %s", configured[c],
                     seen ? "visible" : "NOT VISIBLE - wrong SSID, out of "
                                        "range, or on 5 GHz");
        }
    }

    free(recs);
    esp_wifi_clear_ap_list();
}
