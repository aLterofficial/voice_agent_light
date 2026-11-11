/*
 * Rail monitor - a power-rail voltmeter that does not need a serial port.
 *
 * WHY THIS EXISTS
 * ---------------
 * The board browns out while the WiFi radio powers up. Every measurement so
 * far has gone through the USB serial bridge, and that bridge is itself on the
 * rail that collapses: when the rail goes, the port disappears and the host
 * sees "no port" instead of data (observed repeatedly on 2026-10-02 - COM5
 * and COM7 alternating, then nothing at all for minutes).
 *
 * So the instrument has to be local. This module samples the 3.3V rail through
 * an ADC pin and shows, on an I2C OLED refreshed four times a second:
 *
 *     RAIL MONITOR
 *     now   3.28 V
 *     5s min 2.41 V      <- the number that matters
 *     max   3.34 V
 *     boot min 2.41 V
 *     rst  BROWNOUT x12  <- detector trip, or a full collapse
 *
 * WHAT THE NUMBERS MEAN
 * ---------------------
 * A brownout reset tells you the rail reached the detector's threshold
 * (2.44V, the lowest the ESP32-S3 offers). It does NOT tell you how far it
 * fell or how long it stayed there. The minimum does:
 *
 *   * min near 0V or an immediate restart -> the rail actually collapsed and
 *     the chip lost power; the detector was a bystander
 *   * min about 2.4V and the reading recovers -> a brief dip that only the
 *     detector caught
 *   * min stays above 3.0V yet resets happen -> look elsewhere (reset is not
 *     supply-related)
 *
 * WIRING (nothing is connected to the ADC pin by default)
 * ------------------------------------------------------
 *   3V3 ---[ R1 10k ]---+---[ R2 10k ]--- GND
 *                       |
 *                       +--- RAIL_ADC_GPIO   (ADC1, GPIO1..GPIO10)
 *
 * The divider is required: the ADC's usable input tops out around 3.1V. With
 * R1 = R2 the ratio is 2.00, so a 3.3V rail presents 1.65V. Set
 * CONFIG_RAIL_DIVIDER_X100 if your resistors differ.
 *
 * OLED: 4-pin I2C module (VCC GND SCL SDA). 0.96 inch modules are almost
 * always SSD1306; an SH1106 needs CONFIG_RAIL_OLED_XOFFSET=2. VCC to 3V3,
 * which also means the display browns out with everything else - that is
 * fine, it comes back with the next boot and the boot counter keeps counting.
 *
 * The driver is deliberately minimal (init sequence + 1 KB framebuffer +
 * 5x7 font) rather than pulling a managed component, so this works with no
 * network access - which matters when the parts you ordered are stuck in a
 * holiday courier backlog.
 */

#include <stdio.h>
#include <string.h>

#include "app_internal.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if CONFIG_RAIL_MONITOR

static const char *TAG = "rail";

#define OLED_W 128
#define OLED_H 64
#define OLED_PAGES (OLED_H / 8)

/* One framebuffer page per 8-pixel row: 128 bytes per page, bit 0 = top. */
static uint8_t s_fb[OLED_PAGES * OLED_W];

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_oled = NULL;
static bool s_oled_ok = false;

/*
 * Survives a reset: the whole point is to count resets across brownouts.
 * Cleared only by a power-on (RTC memory keeps its contents through a CPU or
 * brownout reset, but not through a full power cycle).
 */
static RTC_NOINIT_ATTR uint32_t s_boot_count;

/* ------------------------------------------------------------------------ */
/* Minimal SSD1306 / SH1106 driver over I2C                                  */
/* ------------------------------------------------------------------------ */

static esp_err_t oled_cmd(uint8_t cmd)
{
    uint8_t buf[2] = { 0x00, cmd };   /* control byte 0x00 = command stream */
    return i2c_master_transmit(s_oled, buf, sizeof(buf), 100);
}

static esp_err_t oled_cmds(const uint8_t *cmds, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        esp_err_t err = oled_cmd(cmds[i]);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

static esp_err_t oled_flush(void)
{
    /* Each page is: control byte 0x40 (data) followed by 128 pixel bytes. */
    static uint8_t tx[1 + OLED_W];
    const uint8_t xoff = CONFIG_RAIL_OLED_XOFFSET;

    tx[0] = 0x40;
    for (int page = 0; page < OLED_PAGES; page++) {
        /* Set page address, then column address (low nibble first). */
        esp_err_t err = oled_cmds((const uint8_t[]){
            0xB0 | page,
            0x00 | (xoff & 0x0F),
            0x10 | ((xoff >> 4) & 0x0F),
        }, 3);
        if (err != ESP_OK) {
            return err;
        }
        memcpy(&tx[1], &s_fb[page * OLED_W], OLED_W);
        err = i2c_master_transmit(s_oled, tx, sizeof(tx), 100);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

static esp_err_t oled_init(void)
{
    static const uint8_t init_seq[] = {
        0xAE,               /* display off */
        0xD5, 0x80,         /* clock divide / oscillator */
        0xA8, 0x3F,         /* multiplex = 64 */
        0xD3, 0x00,         /* display offset 0 (SH1106 users: see x-offset option) */
        0x40,               /* start line 0 */
        0x8D, 0x14,         /* charge pump on */
        0x20, 0x02,         /* page addressing mode */
        0xA1,               /* segment remap: column 127 -> SEG0 */
        0xC8,               /* COM scan direction remapped */
        0xDA, 0x12,         /* COM pins: alternative, sequential */
        0x81, 0xCF,         /* contrast */
        0xD9, 0xF1,         /* pre-charge */
        0xDB, 0x40,         /* VCOMH deselect */
        0xA4,               /* resume from RAM */
        0xA6,               /* normal (not inverted) */
        0xAF,               /* display on */
    };

    memset(s_fb, 0, sizeof(s_fb));
    esp_err_t err = oled_cmds(init_seq, sizeof(init_seq));
    if (err != ESP_OK) {
        return err;
    }
    return oled_flush();
}

static bool oled_probe(uint8_t addr)
{
    if (i2c_master_probe(s_bus, addr, 100) != ESP_OK) {
        return false;
    }
    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(s_bus, &dev, &s_oled) != ESP_OK) {
        return false;
    }
    ESP_LOGI(TAG, "OLED answered at 0x%02X", addr);
    return true;
}

/*
 * Anything above this is not a rail voltage: the ESP32-S3 rail cannot legally
 * exceed 3.6V. A reading this high means the ADC input is saturated, which is
 * what a floating pin does - with the divider ratio applied it shows up as
 * roughly 6.3V. Screening it out matters because "6.3V" on a 3.3V rail is a
 * nonsense number that would send someone chasing a wiring fault that is not
 * there; "OVR" says "no divider connected yet" instead.
 */
#define RAIL_OVER_RANGE_MV 3550

/* ------------------------------------------------------------------------ */
/* Framebuffer text: 5x7 glyphs, 6 columns per character                      */
/* ------------------------------------------------------------------------ */

/* ASCII 0x20..0x7F, five columns each, bit 0 = top row. */
static const uint8_t *glyph(char c)
{
    static const uint8_t font[96][5] = {
        {0x00,0x00,0x00,0x00,0x00}, /*   */
        {0x00,0x00,0x5F,0x00,0x00}, /* ! */
        {0x00,0x07,0x00,0x07,0x00}, /* " */
        {0x14,0x7F,0x14,0x7F,0x14}, /* # */
        {0x24,0x2A,0x7F,0x2A,0x12}, /* $ */
        {0x23,0x13,0x08,0x64,0x62}, /* % */
        {0x36,0x49,0x55,0x22,0x50}, /* & */
        {0x00,0x05,0x03,0x00,0x00}, /* ' */
        {0x00,0x1C,0x22,0x41,0x00}, /* ( */
        {0x00,0x41,0x22,0x1C,0x00}, /* ) */
        {0x08,0x2A,0x1C,0x2A,0x08}, /* * */
        {0x08,0x08,0x3E,0x08,0x08}, /* + */
        {0x00,0x50,0x30,0x00,0x00}, /* , */
        {0x08,0x08,0x08,0x08,0x08}, /* - */
        {0x00,0x60,0x60,0x00,0x00}, /* . */
        {0x20,0x10,0x08,0x04,0x02}, /* / */
        {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
        {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
        {0x42,0x61,0x51,0x49,0x46}, /* 2 */
        {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
        {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
        {0x27,0x45,0x45,0x45,0x39}, /* 5 */
        {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
        {0x01,0x71,0x09,0x05,0x03}, /* 7 */
        {0x36,0x49,0x49,0x49,0x36}, /* 8 */
        {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
        {0x00,0x36,0x36,0x00,0x00}, /* : */
        {0x00,0x56,0x36,0x00,0x00}, /* ; */
        {0x00,0x08,0x14,0x22,0x41}, /* < */
        {0x14,0x14,0x14,0x14,0x14}, /* = */
        {0x41,0x22,0x14,0x08,0x00}, /* > */
        {0x02,0x01,0x51,0x09,0x06}, /* ? */
        {0x32,0x49,0x79,0x41,0x3E}, /* @ */
        {0x7E,0x11,0x11,0x11,0x7E}, /* A */
        {0x7F,0x49,0x49,0x49,0x36}, /* B */
        {0x3E,0x41,0x41,0x41,0x22}, /* C */
        {0x7F,0x41,0x41,0x22,0x1C}, /* D */
        {0x7F,0x49,0x49,0x49,0x41}, /* E */
        {0x7F,0x09,0x09,0x09,0x01}, /* F */
        {0x3E,0x41,0x49,0x49,0x7A}, /* G */
        {0x7F,0x08,0x08,0x08,0x7F}, /* H */
        {0x00,0x41,0x7F,0x41,0x00}, /* I */
        {0x20,0x40,0x41,0x3F,0x01}, /* J */
        {0x7F,0x08,0x14,0x22,0x41}, /* K */
        {0x7F,0x40,0x40,0x40,0x40}, /* L */
        {0x7F,0x02,0x0C,0x02,0x7F}, /* M */
        {0x7F,0x04,0x08,0x10,0x7F}, /* N */
        {0x3E,0x41,0x41,0x41,0x3E}, /* O */
        {0x7F,0x09,0x09,0x09,0x06}, /* P */
        {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
        {0x7F,0x09,0x19,0x29,0x46}, /* R */
        {0x46,0x49,0x49,0x49,0x31}, /* S */
        {0x01,0x01,0x7F,0x01,0x01}, /* T */
        {0x3F,0x40,0x40,0x40,0x3F}, /* U */
        {0x1F,0x20,0x40,0x20,0x1F}, /* V */
        {0x7F,0x20,0x18,0x20,0x7F}, /* W */
        {0x63,0x14,0x08,0x14,0x63}, /* X */
        {0x03,0x04,0x78,0x04,0x03}, /* Y */
        {0x61,0x51,0x49,0x45,0x43}, /* Z */
        {0x00,0x7F,0x41,0x41,0x00}, /* [ */
        {0x02,0x04,0x08,0x10,0x20}, /* \ */
        {0x00,0x41,0x41,0x7F,0x00}, /* ] */
        {0x04,0x02,0x01,0x02,0x04}, /* ^ */
        {0x40,0x40,0x40,0x40,0x40}, /* _ */
        {0x00,0x01,0x02,0x04,0x00}, /* ` */
        {0x20,0x54,0x54,0x54,0x78}, /* a */
        {0x7F,0x48,0x44,0x44,0x38}, /* b */
        {0x38,0x44,0x44,0x44,0x20}, /* c */
        {0x38,0x44,0x44,0x48,0x7F}, /* d */
        {0x38,0x54,0x54,0x54,0x18}, /* e */
        {0x08,0x7E,0x09,0x01,0x02}, /* f */
        {0x0C,0x52,0x52,0x52,0x3E}, /* g */
        {0x7F,0x08,0x04,0x04,0x78}, /* h */
        {0x00,0x44,0x7D,0x40,0x00}, /* i */
        {0x20,0x40,0x44,0x3D,0x00}, /* j */
        {0x7F,0x10,0x28,0x44,0x00}, /* k */
        {0x00,0x41,0x7F,0x40,0x00}, /* l */
        {0x7C,0x04,0x18,0x04,0x78}, /* m */
        {0x7C,0x08,0x04,0x04,0x78}, /* n */
        {0x38,0x44,0x44,0x44,0x38}, /* o */
        {0x7C,0x14,0x14,0x14,0x08}, /* p */
        {0x08,0x14,0x14,0x18,0x7C}, /* q */
        {0x7C,0x08,0x04,0x04,0x08}, /* r */
        {0x48,0x54,0x54,0x54,0x20}, /* s */
        {0x04,0x3F,0x44,0x40,0x20}, /* t */
        {0x3C,0x40,0x40,0x20,0x7C}, /* u */
        {0x1C,0x20,0x40,0x20,0x1C}, /* v */
        {0x3C,0x40,0x30,0x40,0x3C}, /* w */
        {0x44,0x28,0x10,0x28,0x44}, /* x */
        {0x0C,0x50,0x50,0x50,0x3C}, /* y */
        {0x44,0x64,0x54,0x4C,0x44}, /* z */
        {0x00,0x08,0x36,0x41,0x00}, /* { */
        {0x00,0x00,0x7F,0x00,0x00}, /* | */
        {0x00,0x41,0x36,0x08,0x00}, /* } */
        {0x08,0x08,0x2A,0x1C,0x08}, /* ~ */
        {0x08,0x1C,0x2A,0x08,0x08}, /* (del) */
    };

    if (c < 0x20 || c > 0x7F) {
        c = '?';
    }
    return font[(int)c - 0x20];
}

static void fb_clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
}

static void fb_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H) {
        return;
    }
    uint8_t *p = &s_fb[(y / 8) * OLED_W + x];
    if (on) {
        *p |= (uint8_t)(1u << (y & 7));
    } else {
        *p &= (uint8_t)~(1u << (y & 7));
    }
}

static void fb_text(int x, int y, const char *s)
{
    while (*s) {
        const uint8_t *g = glyph(*s++);
        for (int col = 0; col < 5; col++) {
            uint8_t bits = g[col];
            for (int row = 0; row < 7; row++) {
                if (bits & (1u << row)) {
                    fb_pixel(x + col, y + row, true);
                }
            }
        }
        x += 6;
    }
}

/* ------------------------------------------------------------------------ */
/* Sampling                                                                  */
/* ------------------------------------------------------------------------ */

static adc_oneshot_unit_handle_t s_adc = NULL;
static adc_cali_handle_t s_cali = NULL;
static adc_channel_t s_chan;

/* Shared with the display task; all in millivolts. */
static volatile int s_last_mv;
static volatile int s_win_min_mv;
static volatile int s_win_max_mv;
static volatile int s_boot_min_mv = 0x7FFFFFFF;

/*
 * Last raw conversion, kept for diagnostics. A saturated raw value (near 4095)
 * means the ADC pin is not being driven by anything - i.e. the divider is not
 * actually connected - which is otherwise indistinguishable from a genuinely
 * over-range input.
 */
static volatile int s_last_raw;

/*
 * Calibrated voltage AT THE PIN, before the divider ratio is applied. Kept
 * separate from s_last_mv because the two together localise a wiring fault:
 * the pin voltage says what the ADC actually saw, the rail voltage says what
 * that implies about the divider.
 */
static volatile int s_last_pin_mv;

/*
 * Second view, aimed at the setup phase: the minimum and maximum PIN voltage
 * over the last 10 s. Turn the divider (a pot, usually) through its range and
 * this says whether the reading sweeps smoothly and whether it passes through
 * the 1.65V that a 2:1 divider on a 3.3V rail must produce.
 *
 * Two views rather than one because they answer different questions:
 *   RAIL view  - how far does the rail fall?   (the actual measurement)
 *   TUNE view  - is the divider set up right?  (this one)
 * On a 64-pixel display they cannot both be legible at once, so the board's
 * RGB LED switches between them; the current view is also named in the log.
 */
static volatile int s_tune_min_mv = 0x7FFFFFFF;
static volatile int s_tune_max_mv;
static volatile bool s_tune_mode;
static volatile int s_tune_age_ms;

static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SW:       return "SW";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_INT_WDT:  return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT:      return "WDT";
    case ESP_RST_DEEPSLEEP:return "DEEPSLEEP";
    case ESP_RST_EXT:      return "EXT";
    case ESP_RST_SDIO:     return "SDIO";
    case ESP_RST_USB:      return "USB";
    case ESP_RST_JTAG:     return "JTAG";
    case ESP_RST_EFUSE:    return "EFUSE";
    /* PWR_GLITCH is the interesting one here: the hardware saw the supply dip
     * without the brownout detector reaching its threshold, i.e. a glitch that
     * stayed above 2.44V. That is a different failure mode from BROWNOUT and
     * was previously collapsing into "OTHER", which hid the distinction. */
    case ESP_RST_PWR_GLITCH:return "PWR_GLITCH";
    case ESP_RST_CPU_LOCKUP:return "CPU_LOCKUP";
    default:               return "OTHER";
    }
}

static bool adc_setup(void)
{
    adc_unit_t unit = ADC_UNIT_1;
    /* Translate the configured GPIO into unit+channel rather than hardcoding,
     * so the Kconfig pin can move within GPIO1..GPIO10 freely. */
    if (adc_oneshot_io_to_channel(CONFIG_RAIL_ADC_GPIO, &unit, &s_chan) != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d is not an ADC1 pin", CONFIG_RAIL_ADC_GPIO);
        return false;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = unit };
    if (adc_oneshot_new_unit(&unit_cfg, &s_adc) != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed");
        return false;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_oneshot_config_channel(s_adc, s_chan, &chan_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed");
        return false;
    }
    ESP_LOGI(TAG, "GPIO%d maps to ADC unit %d, channel %d",
             CONFIG_RAIL_ADC_GPIO, (int)unit, (int)s_chan);

    adc_cali_curve_fitting_config_t cal = {
        .unit_id = unit,
        .chan = s_chan,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cal, &s_cali) != ESP_OK) {
        ESP_LOGW(TAG, "no ADC calibration available - readings are approximate");
        s_cali = NULL;
    }

    /*
     * Stability self-test.
     *
     * An earlier version tried to read the chip's internal GND/VREF channels,
     * which no longer exist in IDF v6 (adc_channel_t is only 0..10 now), so the
     * test is done against the configured pin instead: take a handful of
     * readings and report the spread. A pin driven by a divider gives nearly
     * identical values; a floating pin gives a wide spread, and that is the
     * difference between "the divider is not connected" and "the divider is
     * connected but wrong".
     */
    int lo = 4095, hi = 0;
    for (int i = 0; i < 16; i++) {
        int raw = 0;
        if (adc_oneshot_read(s_adc, s_chan, &raw) == ESP_OK) {
            if (raw < lo) { lo = raw; }
            if (raw > hi) { hi = raw; }
        }
    }
    ESP_LOGW(TAG, "self-test on GPIO%d: 16 reads spanned raw %d..%d (spread %d)",
             CONFIG_RAIL_ADC_GPIO, lo, hi, hi - lo);
    if (hi - lo > 300) {
        ESP_LOGE(TAG, "  -> that spread means NOTHING is driving the pin: the "
                      "divider wire is not connected to GPIO%d (or is not making "
                      "contact). A real divider holds the reading steady.",
                 CONFIG_RAIL_ADC_GPIO);
    } else {
        ESP_LOGI(TAG, "  -> reading is stable, so the pin is being driven");
    }

    return true;
}

/*
 * Sample as fast as the oneshot driver allows (~1 kHz) and keep the extremes.
 *
 * Deliberately runs at high priority on the same core as the WiFi task: the
 * dip we are hunting lasts single-digit milliseconds, and anything that yields
 * for tens of milliseconds will simply miss it.
 */
static void sample_task(void *arg)
{
    (void)arg;
    int64_t window_start = esp_timer_get_time();

    while (true) {
        /*
         * Average a few conversions instead of trusting one.
         *
         * This is not cosmetic: the minimum-hold is the point of the
         * instrument, and a single noisy conversion drags the minimum to
         * whatever the worst sample happened to be (measured: a floating input
         * produced raw 0..4095 within 16 reads, which pinned "5smin" at 0 mV
         * forever). Averaging five reads costs about 100 us and removes most of
         * that, while still being far faster than the millisecond-scale dip
         * being hunted.
         */
        int raw = 0;
        int acc = 0;
        int n_ok = 0;
        for (int i = 0; i < 5; i++) {
            if (adc_oneshot_read(s_adc, s_chan, &raw) == ESP_OK) {
                acc += raw;
                n_ok++;
            }
        }
        if (n_ok > 0) {
            raw = acc / n_ok;
            int mv = 0;
            s_last_raw = raw;
            if (s_cali) {
                adc_cali_raw_to_voltage(s_cali, raw, &mv);
            } else {
                /* Rough fallback: 12-bit, 12 dB atten, ~3.1V full scale. */
                mv = raw * 3100 / 4095;
            }
            s_last_pin_mv = mv;   /* before undoing the divider */
            /* Undo the divider to get the rail voltage. */
            mv = mv * CONFIG_RAIL_DIVIDER_X100 / 100;

            /* Tuning view: extremes at the pin over a 10 s window. */
            if (s_last_pin_mv < s_tune_min_mv) { s_tune_min_mv = s_last_pin_mv; }
            if (s_last_pin_mv > s_tune_max_mv) { s_tune_max_mv = s_last_pin_mv; }
            if (++s_tune_age_ms >= 10000) {
                s_tune_age_ms = 0;
                s_tune_min_mv = s_last_pin_mv;
                s_tune_max_mv = s_last_pin_mv;
            }

            s_last_mv = mv;
            if (mv < s_win_min_mv) { s_win_min_mv = mv; }
            if (mv > s_win_max_mv) { s_win_max_mv = mv; }
            if (mv < s_boot_min_mv) { s_boot_min_mv = mv; }
        }

        if (esp_timer_get_time() - window_start > 5000000) {   /* 5 s window */
            window_start = esp_timer_get_time();
            s_win_min_mv = 0x7FFFFFFF;
            s_win_max_mv = 0;
        }
        vTaskDelay(1);   /* 1 tick = 1 ms at the default tick rate */
    }
}

/* ------------------------------------------------------------------------ */
/* Display task                                                              */
/* ------------------------------------------------------------------------ */

static bool setup_led(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << CONFIG_PIN_LED,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&cfg) != ESP_OK) {
        return false;
    }
    gpio_set_level(CONFIG_PIN_LED, 0);
    return true;
}

static void draw_screen(void)
{
    /* Generously sized: -Werror=format-truncation cannot prove a 4-digit
     * millivolt value and a two-decimal rendering both fit in a tight buffer,
     * and the cost of 40 bytes on the stack is irrelevant here. */
    char line[40];

    fb_clear();

    if (s_tune_mode) {
        /* --- divider tuning view ---------------------------------------- */
        int lo = (s_tune_min_mv == 0x7FFFFFFF) ? s_last_pin_mv : s_tune_min_mv;
        int hi = s_tune_max_mv;
        bool in_range = (lo <= 1650 && 1650 <= hi);

        fb_text(0, 0, "DIVIDER TUNE");
        /* One decimal place throughout: 12 pixels per row leaves no room for
         * two 6-character voltages, and 0.1 V is far finer than the question
         * being asked ("did the sweep pass through 1.65 V?"). */
        snprintf(line, sizeof(line), "pin %d.%d V", s_last_pin_mv / 1000, (s_last_pin_mv % 1000) / 100);
        fb_text(0, 14, line);
        snprintf(line, sizeof(line), "10s %d.%d-%d.%d", lo / 1000, (lo % 1000) / 100,
                 hi / 1000, (hi % 1000) / 100);
        fb_text(0, 28, line);
        fb_text(0, 42, in_range ? "PASS 1.65 seen" : "turn to 1.65V");
        oled_flush();
        return;
    }

    /* --- rail voltage view ---------------------------------------------- */
    int win_min = (s_win_min_mv == 0x7FFFFFFF) ? s_last_mv : s_win_min_mv;
    int boot_min = (s_boot_min_mv == 0x7FFFFFFF) ? s_last_mv : s_boot_min_mv;
    bool over = (s_last_mv > RAIL_OVER_RANGE_MV);

    fb_text(0, 0, over ? "RAIL MONITOR OVR" : "RAIL MONITOR");
    if (over) {
        fb_text(0, 22, "input saturated");
        fb_text(0, 32, "no divider yet?");
    } else {
        snprintf(line, sizeof(line), "now   %d.%02d V", s_last_mv / 1000, (s_last_mv % 1000) / 10);
        fb_text(0, 22, line);

        snprintf(line, sizeof(line), "5smin %d.%02d V", win_min / 1000, (win_min % 1000) / 10);
        fb_text(0, 32, line);

        snprintf(line, sizeof(line), "boot  %d.%02d V", boot_min / 1000, (boot_min % 1000) / 10);
        fb_text(0, 42, line);
    }

    snprintf(line, sizeof(line), "%s x%u", reset_reason_str(esp_reset_reason()),
             (unsigned)s_boot_count);
    fb_text(0, 54, line);

    oled_flush();
}

static void display_task(void *arg)
{
    (void)arg;
    while (true) {
        if (s_oled_ok) {
            draw_screen();
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/*
 * The LED doubles as the mode indicator: dark = RAIL view, lit = TUNE view.
 * 20 s per view, so leaving the board alone returns it to the measurement.
 */
static void mode_task(void *arg)
{
    (void)arg;
    while (true) {
        s_tune_mode = false;
        gpio_set_level(CONFIG_PIN_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(20000));
        s_tune_mode = true;
        gpio_set_level(CONFIG_PIN_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(20000));
    }
}

/* ------------------------------------------------------------------------ */
/* Entry point                                                               */
/* ------------------------------------------------------------------------ */

void rail_monitor_run(void)
{
    /*
     * Boot counter. RTC memory survives a brownout or watchdog reset, so a
     * rising count with POWERON/BROWNOUT reasons is itself evidence of how
     * often the rail let go - visible on the display even when the serial
     * port never comes up.
     */
    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_POWERON || s_boot_count == 0xFFFFFFFF) {
        s_boot_count = 1;
    } else {
        s_boot_count++;
    }

    ESP_LOGW(TAG, "=================================================");
    ESP_LOGW(TAG, "  RAIL MONITOR  boot #%u  reset reason: %s",
             (unsigned)s_boot_count, reset_reason_str(reason));
    ESP_LOGW(TAG, "=================================================");

    if (!adc_setup()) {
        ESP_LOGE(TAG, "cannot sample the rail - nothing useful to show");
        return;
    }
    ESP_LOGI(TAG, "sampling the rail on GPIO%d (divider x%d/100)",
             CONFIG_RAIL_ADC_GPIO, CONFIG_RAIL_DIVIDER_X100);

    /* --- Find the display -------------------------------------------------
     * The search itself lives in i2c_scan.c because the servo bench mode needs
     * the same answer ("is the OLED actually wired up?"). This function owns
     * the bus it ends up using, so the scan only reports the pins. */
    int sda = CONFIG_I2C_SDA_GPIO;
    int scl = CONFIG_I2C_SCL_GPIO;
    uint8_t addr = CONFIG_I2C_PROBE_ADDR;
    bool have_oled = false;

#if CONFIG_RAIL_SCAN_I2C_PINS
    const int scan_timeout_ms = 3000;
#else
    const int scan_timeout_ms = 0;      /* configured pins only, no sweep */
#endif

    if (i2c_scan_find_report(CONFIG_I2C_PROBE_ADDR, sda, scl,
                             &sda, &scl, &addr, scan_timeout_ms)) {
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = -1,
            .sda_io_num = sda,
            .scl_io_num = scl,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        if (i2c_new_master_bus(&bus_cfg, &s_bus) == ESP_OK &&
            oled_probe(addr)) {
            have_oled = true;
        }
    }

    if (!have_oled) {
        ESP_LOGE(TAG, "no display - the rail is still being sampled and "
                      "printed to the serial log, which is all that is left");
    } else if (oled_init() != ESP_OK) {
        ESP_LOGE(TAG, "display answered but did not initialise");
        have_oled = false;
    } else {
        s_oled_ok = true;
        ESP_LOGI(TAG, "display ready on SDA=GPIO%d SCL=GPIO%d addr 0x%02X "
                      "(x-offset %d; use 2 for an SH1106)",
                 sda, scl, addr, CONFIG_RAIL_OLED_XOFFSET);
    }

    /* --- run -------------------------------------------------------------- */
    s_last_mv = 0;
    s_win_max_mv = 0;
    s_win_min_mv = 0x7FFFFFFF;

    xTaskCreatePinnedToCore(sample_task, "rail_adc", 3072, NULL, 20, NULL, 0);
    xTaskCreate(display_task, "rail_oled", 4096, NULL, 4, NULL);
    if (setup_led()) {
        xTaskCreate(mode_task, "rail_mode", 2048, NULL, 3, NULL);
        ESP_LOGI(TAG, "LED (GPIO%d) shows the view: dark = rail voltage, "
                      "lit = divider tuning (20 s each)", CONFIG_PIN_LED);
    }

    /* The serial log is a bonus, not the instrument: it prints whatever gets
     * out before the next collapse. 10 s is a compromise - often enough to
     * watch a pot being turned, rare enough not to bury the log. */
    uint32_t n = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        int win_min = (s_win_min_mv == 0x7FFFFFFF) ? s_last_mv : s_win_min_mv;
        int boot_min = (s_boot_min_mv == 0x7FFFFFFF) ? s_last_mv : s_boot_min_mv;
        int tune_lo = (s_tune_min_mv == 0x7FFFFFFF) ? s_last_pin_mv : s_tune_min_mv;
        if (s_last_mv > RAIL_OVER_RANGE_MV) {
            ESP_LOGW(TAG, "#%u  OVER RANGE (raw ADC %d/4095 - nothing is driving "
                          "the pin)   now=%d mV  10s tune %d..%d mV   oled=%s  "
                          "adc=GPIO%d  (uptime %u s)  %s x%u",
                     ++n, s_last_raw, s_last_mv, tune_lo, s_tune_max_mv,
                     s_oled_ok ? "yes" : "NO", CONFIG_RAIL_ADC_GPIO,
                     (unsigned)(esp_timer_get_time() / 1000000),
                     reset_reason_str(reason), (unsigned)s_boot_count);
        } else {
            ESP_LOGW(TAG, "#%u  now %d mV  5smin %d mV  bootmin %d mV  "
                          "(raw %d -> pin %d mV)  10s tune %d..%d mV  oled=%s  "
                          "adc=GPIO%d  (uptime %u s)  %s x%u",
                     ++n, s_last_mv, win_min, boot_min, s_last_raw, s_last_pin_mv,
                     tune_lo, s_tune_max_mv,
                     s_oled_ok ? "yes" : "NO", CONFIG_RAIL_ADC_GPIO,
                     (unsigned)(esp_timer_get_time() / 1000000),
                     reset_reason_str(reason), (unsigned)s_boot_count);
        }
    }
}

#else  /* !CONFIG_RAIL_MONITOR */

void rail_monitor_run(void)
{
}

#endif
