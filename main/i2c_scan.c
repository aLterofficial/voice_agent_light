/*
 * I2C bus discovery - finding which GPIO pair a module is wired to.
 *
 * WHY THIS IS SHARED
 * ------------------
 * It started life inside rail_monitor.c, but the need is not specific to the
 * rail monitor: any firmware in this project may want to answer "is the OLED
 * actually connected, and on which pins?" before the user has committed to a
 * wiring. In particular the servo bench mode had no way to check the display
 * at all, so a blank screen there was indistinguishable from a wiring fault.
 *
 * The scan exists because the pins are a free choice on a breadboard, and the
 * alternative is a recompile-and-reflash per guess - slow, and worse, it makes
 * a wiring mistake look like a firmware bug.
 *
 * Skipped GPIOs, and why:
 *   0, 3, 45, 46  strapping pins - driving them at boot changes boot mode
 *   19, 20        native USB D-/D+
 *   26..37        flash and PSRAM
 *   43, 44        UART0 console
 */

#include <string.h>

#include "app_internal.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

static const char *TAG = "i2cscan";

/* Candidate pins, in the order the scan tries them. Nothing here overlaps the
 * reserved ranges listed above. */
static const int k_pins[] = { 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
                              15, 16, 17, 18, 21, 38, 39, 40, 41, 42, 47, 48 };
#define PIN_COUNT (sizeof(k_pins) / sizeof(k_pins[0]))

static i2c_master_bus_handle_t open_bus(int sda, int scl)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = -1,                     /* auto-select a controller */
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    if (i2c_new_master_bus(&cfg, &bus) != ESP_OK) {
        return NULL;
    }
    return bus;
}

static uint8_t probe_addresses(i2c_master_bus_handle_t bus)
{
    /* 0x08..0x77 is the usable 7-bit range; most display and sensor modules sit
     * between 0x18 and 0x3F. */
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (i2c_master_probe(bus, a, 60) == ESP_OK) {
            return a;
        }
    }
    return 0;
}

bool i2c_scan_find_report(uint8_t start_addr, int start_sda, int start_scl,
                          int *sda_out, int *scl_out, uint8_t *addr_out,
                          int timeout_ms)
{
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    /* --- 1. the pins the caller already believes in ---------------------- */
    if (start_sda >= 0 && start_scl >= 0) {
        i2c_master_bus_handle_t bus = open_bus(start_sda, start_scl);
        if (bus != NULL) {
            uint8_t found = 0;
            /* The configured address first, then the whole range - a module may
             * be strapped to either of the two usual addresses. */
            if (start_addr && i2c_master_probe(bus, start_addr, 100) == ESP_OK) {
                found = start_addr;
            } else {
                found = probe_addresses(bus);
            }
            if (found) {
                i2c_del_master_bus(bus);
                *sda_out = start_sda;
                *scl_out = start_scl;
                *addr_out = found;
                ESP_LOGI(TAG, "I2C device at 0x%02X on the configured pins "
                              "(SDA=GPIO%d SCL=GPIO%d)", found, start_sda, start_scl);
                return true;
            }
            ESP_LOGW(TAG, "nothing answered on the configured pins "
                          "(SDA=GPIO%d SCL=GPIO%d) - searching", start_sda, start_scl);
            i2c_del_master_bus(bus);
        } else {
            ESP_LOGW(TAG, "could not open an I2C bus on SDA=GPIO%d SCL=GPIO%d",
                     start_sda, start_scl);
        }
    }

    /* --- 2. every other plausible pair ---------------------------------- */
    ESP_LOGW(TAG, "scanning GPIO pairs for an I2C device (max %d ms) ...", timeout_ms);

    for (size_t i = 0; i < PIN_COUNT; i++) {
        for (size_t j = 0; j < PIN_COUNT; j++) {
            if (i == j) {
                continue;
            }
            if (esp_timer_get_time() > deadline) {
                ESP_LOGE(TAG, "scan timed out with %u pairs untried - if the module "
                              "is definitely wired, raise the timeout",
                         (unsigned)((PIN_COUNT * (PIN_COUNT - 1)) - (i * PIN_COUNT + j)));
                return false;
            }

            i2c_master_bus_handle_t bus = open_bus(k_pins[i], k_pins[j]);
            if (bus == NULL) {
                continue;
            }
            uint8_t found = probe_addresses(bus);
            if (found) {
                ESP_LOGW(TAG, "  found a device at 0x%02X on SDA=GPIO%d SCL=GPIO%d",
                         found, k_pins[i], k_pins[j]);
                i2c_del_master_bus(bus);
                *sda_out = k_pins[i];
                *scl_out = k_pins[j];
                *addr_out = found;
                return true;
            }
            i2c_del_master_bus(bus);
        }
    }

    ESP_LOGE(TAG, "  nothing answered on any GPIO pair - check SDA/SCL/VCC/GND, "
                  "that the module is powered, and that it has pull-ups");
    return false;
}
