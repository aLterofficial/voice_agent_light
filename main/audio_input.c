/*
 * INMP441 MEMS microphone capture over I2S.
 *
 * The INMP441 emits 24-bit samples, MSB-first, in 32-bit I2S slots. Reading
 * it as 32-bit and shifting right by 14 yields a usable 16-bit sample
 * (24-bit data sits in bits 31..8; we keep the top 16 of those).
 *
 * Wiring (see Kconfig for the pin numbers):
 *
 *   INMP441  SCK   -> ESP32-S3 PIN_I2S_BCLK
 *   INMP441  WS    -> ESP32-S3 PIN_I2S_WS
 *   INMP441  SD    -> ESP32-S3 PIN_I2S_MIC_DATA
 *   INMP441  L/R   -> GND   (selects the left channel)
 *   INMP441  VDD   -> 3V3   (NOT 5V)
 *   INMP441  GND   -> GND
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "app_internal.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "audio_in";

static i2s_chan_handle_t s_rx = NULL;
static audio_format_t s_fmt = {
    .sample_rate = 0,
    .bytes_per_sample = 2,
};

/* ------------------------------------------------------------------------ */

esp_err_t audio_input_init(void)
{
    s_fmt.sample_rate = app_cfg_audio_sample_rate();

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    /* A deep DMA buffer smooths over WiFi-related scheduling jitter, which is
     * the usual cause of drop-outs in ADC/I2S audio on this family. */
    chan_cfg.dma_desc_num = 8;
    chan_cfg.dma_frame_num = 240;
    chan_cfg.auto_clear = true;

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &s_rx));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(s_fmt.sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = CONFIG_PIN_I2S_BCLK,
            .ws = CONFIG_PIN_I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = CONFIG_PIN_I2S_MIC_DATA,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    /* The INMP441 needs a 32-bit slot with the 24-bit sample left-aligned.
     * Philips format plus a 32-bit slot and the left slot selected is the
     * combination that works; we then shift the captured 32-bit words down
     * to 16 bits in read_pcm16(). */
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_rx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx));

    ESP_LOGI(TAG, "INMP441 ready: %d Hz, mono, 32-bit slots -> 16-bit out",
             (int)s_fmt.sample_rate);
    ESP_LOGI(TAG, "  bclk=GPIO%d ws=GPIO%d din=GPIO%d",
             CONFIG_PIN_I2S_BCLK, CONFIG_PIN_I2S_WS, CONFIG_PIN_I2S_MIC_DATA);
    return ESP_OK;
}

const audio_format_t *audio_input_format(void)
{
    return &s_fmt;
}

/*
 * Stop the RX channel (radio bring-up only).
 *
 * The microphone's bit clock runs continuously whether or not anyone is
 * recording, and its crystal oscillator plus the I2S peripheral's own
 * switching are part of the baseline load when the radio powers up. Idling
 * the channel removes them. The channel is kept, not deleted, so resume is
 * cheap and the DMA descriptors stay valid.
 *
 * Note this is a power measure, not a fix - see radio_bringup.c.
 */
esp_err_t audio_input_stop(void)
{
    if (s_rx == NULL) {
        return ESP_OK;
    }
    esp_err_t err = i2s_channel_disable(s_rx);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2s_channel_disable(RX) failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t audio_input_resume(void)
{
    if (s_rx == NULL) {
        return ESP_OK;
    }
    esp_err_t err = i2s_channel_enable(s_rx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable(RX) failed: %s", esp_err_to_name(err));
    }
    return err;
}

/* ------------------------------------------------------------------------ */

/*
 * Read `want` bytes of 16-bit mono PCM into `dst`.
 * Internally reads 32-bit slots and converts.
 */
static esp_err_t read_pcm16(void *dst, size_t want, size_t *got)
{
    /* 16-bit out <- 32-bit in, so we need want*2 bytes of raw data */
    const size_t raw_want = want * 2;
    uint8_t *raw = malloc(raw_want);
    if (raw == NULL) {
        return ESP_ERR_NO_MEM;
    }

    size_t raw_got = 0;
    esp_err_t err = i2s_channel_read(s_rx, raw, raw_want, &raw_got, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        free(raw);
        return err;
    }

    int16_t *out = (int16_t *)dst;
    const int32_t *in = (const int32_t *)raw;
    const size_t n = raw_got / sizeof(int32_t);
    for (size_t i = 0; i < n; i++) {
        /*
         * 24-bit sample left-justified in bits 31..8, so bits 31..16 are the
         * top 16 bits of it.
         *
         * The shift sets the gain: 14 is 4x, 13 is 8x (see CONFIG_AUDIO_MIC_SHIFT).
         * 8x was measured to break recognition completely - every utterance
         * came back as an empty transcript - because the peaks clip and what
         * reaches the model is no longer speech. 4x recognises reliably.
         *
         * Saturation, not a plain cast: (int16_t) on a value above 32767 wraps
         * the sign bit, turning loud syllables into broadband noise.
         */
        int32_t v = (int32_t)(in[i] >> CONFIG_AUDIO_MIC_SHIFT);
        if (v > 32767) {
            v = 32767;
        } else if (v < -32768) {
            v = -32768;
        }
        out[i] = (int16_t)v;
    }

    free(raw);
    *got = n * sizeof(int16_t);
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */

static int rms_of(const int16_t *s, size_t n)
{
    if (n == 0) {
        return 0;
    }
    /* int64 accumulator: 16k samples of 32767^2 overflows int32 */
    int64_t acc = 0;
    for (size_t i = 0; i < n; i++) {
        acc += (int32_t)s[i] * (int32_t)s[i];
    }
    return (int)sqrt((double)(acc / (int64_t)n));
}

esp_err_t audio_input_record(void *out_buf, size_t buf_capacity, size_t *out_len,
                             int max_ms, int silence_rms, int silence_ms)
{
    if (out_buf == NULL || out_len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_len = 0;

    const int chunk_ms = 30; /* ~one frame; small enough for a fine VAD */
    const size_t bytes_per_ms = (s_fmt.sample_rate * s_fmt.bytes_per_sample) / 1000;
    const size_t chunk_bytes = bytes_per_ms * chunk_ms;

    uint8_t *dst = (uint8_t *)out_buf;
    size_t written = 0;
    int silent_ms_acc = 0;
    int total_ms = 0;
    bool voice_seen = false;

    while (total_ms < max_ms && written + chunk_bytes <= buf_capacity) {
        size_t got = 0;
        esp_err_t err = read_pcm16(dst + written, chunk_bytes, &got);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "i2s read failed: %s", esp_err_to_name(err));
            break;
        }
        written += got;
        total_ms += chunk_ms;

        const int level = rms_of((const int16_t *)(dst + written - got), got / 2);

        if (level >= silence_rms) {
            voice_seen = true;
            silent_ms_acc = 0;
        } else if (voice_seen) {
            silent_ms_acc += chunk_ms;
            if (silent_ms_acc >= silence_ms) {
                ESP_LOGD(TAG, "silence for %d ms -> end of utterance", silent_ms_acc);
                break;
            }
        }
        /* If no voice has been seen yet we keep going until max_ms, so a
         * user who pauses before speaking is not cut off. */
    }

    *out_len = written;
    ESP_LOGI(TAG, "recorded %u bytes (%.1f s)", (unsigned)written,
             (double)written / (s_fmt.sample_rate * s_fmt.bytes_per_sample));
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */

esp_err_t audio_input_wait_for_sound(int trigger_rms, int timeout_ms)
{
    const size_t chunk_bytes = 960; /* 30 ms at 16 kHz */
    uint8_t buf[960];
    const int chunk_ms = 30;
    int waited = 0;

    /* Drain whatever is already buffered so a stale burst does not trigger
     * the wake detector immediately after we start listening. */
    size_t flushed = 0;
    while (i2s_channel_read(s_rx, buf, sizeof(buf), &flushed,
                            pdMS_TO_TICKS(10)) == ESP_OK && flushed > 0) {
        /* keep draining */
    }

    /*
     * Periodic level report. Without it, "nothing happens" is unanswerable: the
     * threshold could be too high, the microphone could be too quiet, or the
     * capture could simply be silent. One line every five seconds turns a guess
     * into a number.
     */
    int peak = 0;
    int since_report = 0;

    while (timeout_ms == 0 || waited < timeout_ms) {
        size_t got = 0;
        esp_err_t err = read_pcm16(buf, chunk_bytes, &got);
        if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            waited += 10;
            continue;
        }
        const int level = rms_of((const int16_t *)buf, got / 2);
        if (level > peak) {
            peak = level;
        }
        if (level >= trigger_rms) {
            ESP_LOGI(TAG, "sound detected (rms %d >= %d)", level, trigger_rms);
            return ESP_OK;
        }
        since_report += chunk_ms;
        if (since_report >= 5000) {
            ESP_LOGI(TAG, "mic level: peak rms %d over 5 s (trigger %d)",
                     peak, trigger_rms);
            peak = 0;
            since_report = 0;
        }
        waited += chunk_ms;
    }
    return ESP_ERR_TIMEOUT;
}
