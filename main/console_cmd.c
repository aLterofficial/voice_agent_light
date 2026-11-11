/*
 * Serial console commands, for diagnosing a venue without a laptop rebuild.
 *
 * Why this exists: at a competition the interesting question is never "does\n * the code compile", it is "why will this thing not join the phone hotspot".
 * The answer is almost always that the hotspot is broadcasting on 5 GHz, and
 * the ESP32-S3 radio cannot see it at all. The 'w' command answers that in
 * one line by listing what the radio can actually hear.
 *
 *   w  scan and list visible APs (channel, RSSI, security)
 *   n  show the configured networks
 *   i  show connection state, IP, RSSI, heap, reset reason
 *   r  drop the current link and reconnect now
 *   x  restart the chip
 *   h  help
 *
 * A note on the UART driver. Under ESP-IDF v6 with CONFIG_ESP_CONSOLE_UART_*,
 * the console only registers a VFS device; no UART driver is installed. That
 * means uart_read_bytes() fails with "uart driver error" and polls return -1.
 * This module therefore installs the driver itself, exactly as IDF's own
 * esp_console_repl_uart() does, before touching uart_read_bytes().
 * (uart_driver_install() otherwise refuses UART0 when the GDB stub is
 * compiled in; CONFIG_ESP_SYSTEM_GDBSTUB_RUNTIME is off in this project.)
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_internal.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "mbedtls/base64.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "console";

/* ------------------------------------------------------------------------ */

static const char *reset_reason_text(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_EXT:       return "external pin";
    case ESP_RST_SW:        return "software restart";
    case ESP_RST_PANIC:     return "panic / exception";
    case ESP_RST_INT_WDT:   return "interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "task watchdog";
    case ESP_RST_WDT:       return "other watchdog";
    case ESP_RST_DEEPSLEEP: return "woke from deep sleep";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (supply dipped)";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "unknown";
    }
}

static void print_networks(void)
{
    const char *ssids[CONFIG_WIFI_CANDIDATE_MAX];

    /* Reuse the WiFi manager's view of which slots are actually in use. */
    wifi_mgr_describe_candidates(ssids, CONFIG_WIFI_CANDIDATE_MAX);

    printf("\n");
    printf("  configured networks (tried in this order)\n");
    printf("  ----------------------------------------\n");
    int shown = 0;
    for (int i = 0; i < CONFIG_WIFI_CANDIDATE_MAX; i++) {
        if (ssids[i] != NULL) {
            printf("  %d. \"%s\"\n", ++shown, ssids[i]);
        }
    }
    if (shown == 0) {
        printf("  (none - run configure.ps1 or idf.py menuconfig)\n");
    }
    printf("\n  note: a phone hotspot must be 2.4 GHz. Use 'w' to check what\n");
    printf("  the radio can see.\n\n");
}

static void print_status(void)
{
    printf("\n");

    /* --- link --- */
    if (wifi_mgr_is_connected()) {
        wifi_ap_record_t ap = {0};
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            printf("  wifi     : connected to \"%s\", channel %u, %d dBm\n",
                   ap.ssid, ap.primary, ap.rssi);
        } else {
            printf("  wifi     : connected\n");
        }

        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif != NULL) {
            esp_netif_ip_info_t ip = {0};
            if (esp_netif_get_ip_info(netif, &ip) == ESP_OK) {
                printf("  ip       : " IPSTR ", gw " IPSTR "\n",
                       IP2STR(&ip.ip), IP2STR(&ip.gw));
            }
        }
    } else {
        printf("  wifi     : NOT connected\n");
    }

    /* --- memory ---
     * The casts keep the printf format specifiers honest: on Xtensa PRIu32
     * expands to "lu" while size_t is "unsigned int", so mixing the two
     * trips -Werror=format. */
    printf("  heap     : %" PRIu32 " KB internal free\n",
           (uint32_t)(esp_get_free_heap_size() / 1024));
    printf("  heap min : %" PRIu32 " KB (lowest since boot)\n",
           (uint32_t)(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024));

    /* --- boot --- */
    printf("  uptime   : %" PRIu32 " s\n",
           (uint32_t)(esp_timer_get_time() / 1000000));
    printf("  reset    : %s\n", reset_reason_text(esp_reset_reason()));

    /* --- radio power --- */
    int8_t tx = 0;
    if (esp_wifi_get_max_tx_power(&tx) == ESP_OK) {
        printf("  tx power : %.2f dBm\n", tx * 0.25);
    }
#if CONFIG_LOW_POWER_MODE
    printf("  low power: ENABLED (cap %d dBm)\n", CONFIG_LOW_POWER_TX_POWER_DBM);
#else
    printf("  low power: off\n");
#endif

    if (esp_reset_reason() == ESP_RST_BROWNOUT) {
        printf("\n  !! the last boot was a BROWNOUT reset - the 3V3 rail\n");
        printf("     collapsed. Fix the supply before trusting any of this.\n");
        printf("     See BROWNOUT-DIAGNOSIS.md. Note that a brownout whose\n");
        printf("     hint fails to persist shows up as RTC_SW_SYS_RST instead.\n");
    }

    printf("\n");
}

static void print_help(void)
{
    printf("\n");
    printf("  +------------------------------------------+\n");
    printf("  |     ESP32-S3 voice assistant console     |\n");
    printf("  +------------------------------------------+\n");
    printf("   w  scan and list visible access points\n");
    printf("   n  show the configured networks\n");
    printf("   i  connection / memory / reset status\n");
    printf("   r  reconnect now\n");
    printf("   x  restart the chip\n");
#if CONFIG_SERVO_ENABLE
    printf("  -- servo (panel switch), signal on GPIO%d --\n", CONFIG_SERVO_GPIO);
    printf("   a  go to position A (%d deg)\n", CONFIG_SERVO_POS_A_DEG);
    printf("   b  go to position B (%d deg)\n", CONFIG_SERVO_POS_B_DEG);
    printf("   t  toggle A <-> B\n");
    printf("   ,  jog 5 deg towards the minimum\n");
    printf("   .  jog 5 deg towards the maximum   <- use these to find the stops\n");
    printf("   p  release (stop pulses: no holding torque, no standing current)\n");
    printf("   s  servo status\n");
#endif
    printf("   h  help\n");
    printf("  +------------------------------------------+\n");
    printf("  NOTE: a phone hotspot must be 2.4 GHz.\n");
#if CONFIG_SERVO_ENABLE
    printf("  NOTE: the servo must be powered from 5V on its\n");
    printf("        own branch with a bulk capacitor, not 3V3.\n");
#endif
    printf("\n");
    printf("assistant> ");
    fflush(stdout);
}

#if CONFIG_SERVO_ENABLE
/*
 * Servo diagnostics.
 *
 * The jog commands exist because the endpoints cannot be guessed: the servo's
 * mechanical limits and the switch's two stable positions both have to be found
 * on the bench. Jogging 5 degrees at a time and watching when the arm stops
 * moving (or starts buzzing) is how CONFIG_SERVO_ANGLE_MIN/MAX_DEG and
 * CONFIG_SERVO_POS_A/B_DEG get their real values.
 *
 * A buzzing servo is stalled and drawing stall current, so the advice printed
 * here is deliberately blunt: stop, note the angle, back off a few degrees.
 */
static void servo_jog(int delta)
{
    int want = servo_get_angle() + delta;
    if (want < CONFIG_SERVO_ANGLE_MIN_DEG || want > CONFIG_SERVO_ANGLE_MAX_DEG) {
        printf("  %d deg is outside the configured range %d..%d - adjust\n",
               want, CONFIG_SERVO_ANGLE_MIN_DEG, CONFIG_SERVO_ANGLE_MAX_DEG);
        printf("  CONFIG_SERVO_ANGLE_MIN_DEG / MAX_DEG if the arm can really go there.\n");
        return;
    }
    printf("  moving to %d deg ...\n", want);
    esp_err_t err = servo_move_smooth(want);
    if (err != ESP_OK) {
        printf("  move failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("  at %d deg. If it is buzzing, it is stalled: back off 5 deg and\n",
           servo_get_angle());
    printf("  use the angle where it just stops as the endpoint.\n");
}

static void servo_status(void)
{
    printf("\n  servo\n");
    printf("    signal pin  : GPIO%d @ %d Hz\n", CONFIG_SERVO_GPIO, CONFIG_SERVO_FREQ_HZ);
    printf("    pulse range : %d..%d us\n",
           CONFIG_SERVO_PULSE_MIN_US, CONFIG_SERVO_PULSE_MAX_US);
    printf("    angle range : %d..%d deg\n",
           CONFIG_SERVO_ANGLE_MIN_DEG, CONFIG_SERVO_ANGLE_MAX_DEG);
    printf("    switch A/B  : %d / %d deg\n",
           CONFIG_SERVO_POS_A_DEG, CONFIG_SERVO_POS_B_DEG);
    printf("    ramp        : %d deg every %d ms\n",
           CONFIG_SERVO_STEP_DEG, CONFIG_SERVO_STEP_MS);
    printf("    position    : %d deg\n", servo_get_angle());
}
#endif

/* ------------------------------------------------------------------------ */

static void console_task(void *arg)
{
    while (1) {
        uint8_t ch = 0;
        /* Short timeout so the task stays responsive but does not spin. */
        int n = uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(100));
        if (n != 1) {
            continue;
        }

        /* Swallow CR/LF so both CRLF and LF terminals behave. */
        if (ch == '\r' || ch == '\n') {
            continue;
        }

        printf("%c\n", ch);

        switch (ch) {
        case 'w':
        case 'W':
            wifi_mgr_scan_and_report();
            break;

        case 'n':
        case 'N':
            print_networks();
            break;

        case 'i':
        case 'I':
            print_status();
            break;

        case 'r':
        case 'R':
            printf("  -> reconnecting...\n");
            fflush(stdout);
            wifi_mgr_reconnect_now();
            break;

        case 'x':
            printf("  -> restarting\n");
            fflush(stdout);
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
            break;

        case 'h':
        case 'H':
        case '?':
            print_help();
            return; /* print_help already redrew the prompt */

#if CONFIG_SERVO_ENABLE
        case 'a':
        case 'A':
            printf("  -> position A (%d deg)\n", CONFIG_SERVO_POS_A_DEG);
            if (servo_to_a() != ESP_OK) {
                printf("  servo not ready (is CONFIG_SERVO_ENABLE set and init'd?)\n");
            }
            break;

        case 'b':
        case 'B':
            printf("  -> position B (%d deg)\n", CONFIG_SERVO_POS_B_DEG);
            if (servo_to_b() != ESP_OK) {
                printf("  servo not ready\n");
            }
            break;

        case 't':
        case 'T':
            if (servo_toggle() != ESP_OK) {
                printf("  servo not ready\n");
            }
            break;

        case ',':
            servo_jog(-5);
            break;

        case '.':
            servo_jog(5);
            break;

        case 'p':
        case 'P':
            /* Useful when the switch holds itself: a servo holding position
             * draws current continuously, and this supply cannot spare it. */
            if (servo_release() != ESP_OK) {
                printf("  servo not ready\n");
            }
            break;

        case 's':
        case 'S':
            servo_status();
            break;

        case 'v':
        case 'V': {
            /*
             * Dump the microphone as base64 PCM.
             *
             * "STT returns empty" has two very different causes - too quiet, or
             * corrupted data - and the rms number cannot tell them apart. This
             * sends the actual samples to the console so the recording can be
             * decoded into a WAV and listened to on the PC.
             */
            const int rate = app_cfg_audio_sample_rate();
            const size_t want = (size_t)rate * 2 * 2;   /* 2 s */
            uint8_t *buf = heap_caps_malloc(want, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (buf == NULL) {
                printf("  no memory for %u bytes\n", (unsigned)want);
                break;
            }
            printf("  waiting for sound, then recording 2 s ...\n");
            fflush(stdout);
            if (audio_input_wait_for_sound(CONFIG_WAKE_TRIGGER_RMS, 15000) != ESP_OK) {
                printf("  no sound detected\n");
                free(buf);
                break;
            }
            size_t got = 0;
            esp_err_t e = audio_input_record(buf, want, &got, 2500, 400, 400);
            if (e != ESP_OK || got == 0) {
                printf("  record failed: %s\n", esp_err_to_name(e));
                free(buf);
                break;
            }
            printf("  MICDUMP %d %u %d\n", rate, (unsigned)got, 16);
            const size_t cap = (got * 4) / 3 + 8;
            unsigned char *b64 = malloc(cap);
            size_t blen = 0;
            if (b64 == NULL ||
                mbedtls_base64_encode(b64, cap, &blen, buf, got) != 0) {
                printf("  encode failed\n");
                free(b64);
                free(buf);
                break;
            }
            for (size_t i = 0; i < blen; i += 200) {
                size_t chunk = blen - i;
                if (chunk > 200) chunk = 200;
                fwrite(b64 + i, 1, chunk, stdout);
                fputc('\n', stdout);
            }
            printf("  MICEND\n");
            fflush(stdout);
            free(b64);
            free(buf);
            break;
        }
#endif

        default:
            printf("  ?? unknown key '%c' (press h)\n", ch);
            break;
        }

        printf("assistant> ");
        fflush(stdout);
    }
}

/* ------------------------------------------------------------------------ */

esp_err_t console_cmd_init(void)
{
    /*
     * Install the UART driver for the console port.
     *
     * Without this, uart_read_bytes() returns -1 with "uart driver error":
     * ESP-IDF v6 registers the console UART with the VFS but does not install
     * a driver for it. The driver is what provides the interrupt-driven RX
     * ring buffer that uart_read_bytes() reads from.
     *
     * The port's pins and baud rate were already set up by the second-stage
     * bootloader, so only the driver needs installing; the baud rate is
     * re-applied here to match the configured value explicitly.
     */
    const uart_config_t cfg = {
        .baud_rate  = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM,
                                        /* rx */ 1024, /* tx */ 0,
                                        /* no event queue */ 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cannot install the UART driver (%s) - the console "
                      "commands will not be available", esp_err_to_name(err));
        return err;
    }

    ESP_ERROR_CHECK(uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &cfg));
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    if (xTaskCreate(console_task, "console", 4096, NULL, 3, NULL) != pdPASS) {
        ESP_LOGW(TAG, "cannot start the console task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "console ready - press 'h' for commands, 'w' to scan");
    return ESP_OK;
}
