/*
 * On-board addressable RGB LED used as a status indicator.
 *
 *   dim blue     idle, waiting for the wake word
 *   green pulse  listening
 *   amber        thinking (network round trip)
 *   cyan         speaking
 *   purple       connecting to WiFi
 *   red          error
 */

#include "app_internal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "led";

static led_strip_handle_t s_strip = NULL;
static volatile led_state_t s_state = LED_OFF;

/* Pattern to restore after the radio bring-up window; see led_status_quiet(). */
static volatile led_state_t s_quiet_restore = LED_OFF;

static void set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    if (s_strip == NULL) {
        return;
    }
    led_strip_set_pixel(s_strip, 0, r, g, b);
    led_strip_refresh(s_strip);
}

/* A background task animates the non-static states. */
static void led_task(void *arg)
{
    uint8_t phase = 0;

    while (1) {
        switch (s_state) {
        case LED_OFF:
            set_rgb(0, 0, 0);
            break;

        case LED_IDLE:
            /* slow, dim breathing blue */
            phase += 2;
            {
                uint8_t v = (phase < 128) ? phase : (255 - phase);
                set_rgb(0, 0, (uint8_t)(v / 12));
            }
            break;

        case LED_LISTENING:
            /* energetic green - the user can see the device is hearing them */
            phase += 10;
            {
                uint8_t v = (phase < 128) ? phase : (255 - phase);
                set_rgb(0, (uint8_t)(v / 6), 0);
            }
            break;

        case LED_THINKING:
            /* amber sweep */
            phase += 8;
            {
                uint8_t v = (phase < 128) ? phase : (255 - phase);
                set_rgb((uint8_t)(v / 5), (uint8_t)(v / 14), 0);
            }
            break;

        case LED_SPEAKING:
            set_rgb(0, 12, 12);
            break;

        case LED_WIFI:
            set_rgb(8, 0, 10);
            break;

        case LED_ERROR:
            /* fast red heartbeat so it is obvious something is wrong */
            phase += 20;
            {
                bool on = (phase % 64) < 32;
                set_rgb(on ? 24 : 0, 0, 0);
            }
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

esp_err_t led_status_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_PIN_LED,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip);
    if (err != ESP_OK) {
        /* A missing status LED should not stop the assistant working. */
        ESP_LOGW(TAG, "no addressable LED on GPIO%d (%s) - continuing without it",
                 CONFIG_PIN_LED, esp_err_to_name(err));
        s_strip = NULL;
        return ESP_OK;
    }

    led_strip_clear(s_strip);
    xTaskCreate(led_task, "led", 2560, NULL, 3, NULL);
    ESP_LOGI(TAG, "status LED on GPIO%d", CONFIG_PIN_LED);
    return ESP_OK;
}

void led_status_set(led_state_t state)
{
    s_state = state;
}

void led_status_blink_error(void)
{
    s_state = LED_ERROR;
}

/*
 * Blank the LED while the radio powers up, then hand it back (radio bring-up
 * only - see radio_bringup.c).
 *
 * The task keeps running and keeps being driven; it is only the pattern that
 * changes. A single dark WS2812 draws a few milliamps instead of the tens it
 * draws when lit, and - more relevant here - it stops being a switching load
 * during the exact window in which the MAC/PHY domain comes up.
 *
 * The previous state is remembered rather than assumed, so the boot-time
 * "connecting" indication reappears after the radio is up.
 */
void led_status_quiet(void)
{
    s_quiet_restore = s_state;
    s_state = LED_OFF;
    if (s_strip != NULL) {
        led_strip_clear(s_strip);
    }
}

void led_status_unquiet(void)
{
    s_state = s_quiet_restore;
}
