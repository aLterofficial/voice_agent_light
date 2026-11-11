/*
 * MAX98357A I2S amplifier output.
 *
 * Uses I2S_NUM_1 so the microphone keeps I2S_NUM_0 entirely to itself. BCLK
 * and WS are deliberately shared with the microphone - that is the normal way
 * to wire an I2S codec pair - which makes IDF log
 *
 *     W i2s_common: GPIO n is not usable, maybe conflict with others
 *
 * once per shared pin. The warning comes from esp_gpio_reserve() seeing the
 * pin already claimed by the other I2S controller; it is harmless and can be
 * ignored. (See i2s_common.c: esp_gpio_reserve() returning a bit that is
 * already set.)
 *
 * Wiring:
 *
 *   MAX98357A  BCLK -> ESP32-S3 PIN_I2S_BCLK   (shared with the mic)
 *   MAX98357A  LRC  -> ESP32-S3 PIN_I2S_WS     (shared with the mic)
 *   MAX98357A  DIN  -> ESP32-S3 PIN_I2S_AMP_DATA
 *   MAX98357A  SD   -> ESP32-S3 PIN_AMP_ENABLE (high = unmuted)
 *   MAX98357A  VIN  -> 3V3  (5V is also fine and gives more headroom)
 *   MAX98357A  GND  -> GND
 */

#include <stdlib.h>
#include <string.h>

#include "app_internal.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "audio_out";

/* Block-timing accumulators, reset at the start of every utterance. */
static int64_t s_t_first = 0;

/*
 * Pre-roll.
 *
 * Measured on the phone hotspot the downlink goes quiet for up to ~870 ms at a
 * time while the server is still producing at ~2x realtime. The DMA alone
 * cannot cover that for a sentence longer than its depth, so the first chunks
 * are held here and playback does not start until this much is banked. Uneven
 * arrival then costs nothing audible; it only costs startup latency.
 */
static uint8_t *s_pre = NULL;
static size_t   s_pre_cap = 0;
static size_t   s_pre_len = 0;
static bool     s_playing = false;

static i2s_chan_handle_t s_tx = NULL;

esp_err_t audio_output_init(void)
{
    /* --- amplifier enable pin --- */
    if (CONFIG_PIN_AMP_ENABLE >= 0) {
        gpio_config_t en = {
            .pin_bit_mask = 1ULL << CONFIG_PIN_AMP_ENABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&en));
        /* Start muted; the assistant unmutes only while speaking, so idle
         * hiss from the amplifier is not audible. */
        gpio_set_level(CONFIG_PIN_AMP_ENABLE, 0);
    }

    /*
     * --- I2S TX, on its own clock ---
     *
     * The amplifier used to share BCLK/WS with the microphone. Two I2S
     * peripherals cannot both drive a pin, so one had to be a slave - and a
     * slave TX only samples the incoming clock, with no guaranteed phase
     * relationship to its own data output. On this hardware that showed up as
     * hiss riding on a pure 440 Hz tone, and it survived every software change
     * (gain, slot mask, streaming vs buffered). Two spare pins fix it properly.
     *
     * MASTER, 16-bit, stereo slots: the plain MAX98357A configuration.
     */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    /*
     * 16 x 1024 frames = 16384 frames = 682 ms at 24 kHz.
     *
     * Total slack in the chain is socket buffer + HTTP buffer + this. Measured
     * delivery stalls on a phone hotspot reach ~600 ms, which drained the old
     * 341 ms and produced an audible gap mid-sentence. 682 ms keeps playing
     * through that. Costs 64 KB of internal RAM.
     */
    chan_cfg.dma_desc_num = 16;
    chan_cfg.dma_frame_num = 1024;
    chan_cfg.auto_clear = true;

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx, NULL));

    const int rate = app_cfg_audio_out_sample_rate();

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = CONFIG_PIN_I2S_AMP_BCLK,
            .ws = CONFIG_PIN_I2S_AMP_WS,
            .dout = CONFIG_PIN_I2S_AMP_DATA,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx));

    ESP_LOGI(TAG, "MAX98357A ready: %d Hz, 16-bit stereo slots, own clock", rate);
    ESP_LOGI(TAG, "  bclk=GPIO%d ws=GPIO%d dout=GPIO%d en=GPIO%d",
             CONFIG_PIN_I2S_AMP_BCLK, CONFIG_PIN_I2S_AMP_WS,
             CONFIG_PIN_I2S_AMP_DATA, CONFIG_PIN_AMP_ENABLE);
    return ESP_OK;
}

esp_err_t audio_output_set_enabled(bool enabled)
{
    if (CONFIG_PIN_AMP_ENABLE < 0) {
        return ESP_OK;
    }
    gpio_set_level(CONFIG_PIN_AMP_ENABLE, enabled ? 1 : 0);
    if (enabled) {
        /* The MAX98357A needs a moment to come out of shutdown before it
         * will reproduce the first samples faithfully. */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_OK;
}

/*
 * Stop the TX channel and hard-mute the amplifier (radio bring-up only).
 *
 * Order matters twice over. The enable pin goes low FIRST so the amplifier is
 * in shutdown before the bit clock stops - stopping the clock while the part
 * is enabled leaves it in an undefined state and it can click. And because
 * GPIO18 is driven low explicitly rather than left floating, the amplifier
 * cannot contribute anything to the current the radio's power-up sees.
 *
 * See radio_bringup.c for why this is done at all.
 */
esp_err_t audio_output_stop(void)
{
    esp_err_t first_err = ESP_OK;

    if (CONFIG_PIN_AMP_ENABLE >= 0) {
        gpio_set_level(CONFIG_PIN_AMP_ENABLE, 0);
    }
    if (s_tx == NULL) {
        return ESP_OK;
    }
    esp_err_t err = i2s_channel_disable(s_tx);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2s_channel_disable(TX) failed: %s", esp_err_to_name(err));
        first_err = err;
    }
    return first_err;
}

esp_err_t audio_output_resume(void)
{
    if (s_tx == NULL) {
        return ESP_OK;
    }
    esp_err_t err = i2s_channel_enable(s_tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable(TX) failed: %s", esp_err_to_name(err));
        return err;
    }
    /* Stay muted: the assistant unmutes only while speaking. */
    return ESP_OK;
}

esp_err_t audio_output_stream_begin(void)
{
    if (s_tx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_t_first = 0;
    s_pre_len = 0;
    s_playing = false;

    const int rate = app_cfg_audio_out_sample_rate();
    const size_t want = (size_t)rate * 2 * CONFIG_AUDIO_OUT_PREROLL_MS / 1000;
    if (want > 0 && (s_pre == NULL || s_pre_cap < want)) {
        uint8_t *nb = realloc(s_pre, want);
        if (nb == NULL) {
            ESP_LOGW(TAG, "no memory for a %d ms pre-roll - playing as it arrives",
                     CONFIG_AUDIO_OUT_PREROLL_MS);
            s_pre_cap = 0;
        } else {
            s_pre = nb;
            s_pre_cap = want;
        }
    }
    /* The amplifier stays muted until there is enough audio to finish a phrase
     * without a gap. */
    return ESP_OK;
}

static esp_err_t raw_write(const void *pcm16, size_t len)
{
    if (s_tx == NULL || pcm16 == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Two things here are load-bearing, both learned the hard way.
     *
     * 1. The mono sample goes into the LEFT slot and zero into the right. The
     *    amplifier sums the two channels, so writing the sample twice doubles
     *    the amplitude (+6 dB) and clips everything that was already loud -
     *    audible as harsh, buzzy distortion rather than clean audio.
     *
     * 2. The byte stream is reassembled across calls. HTTP delivers audio in
     *    chunks whose length is not guaranteed to be even, and every chunk is
     *    handed to this function as it arrives. Dropping a stray odd byte
     *    shifts the whole remainder of the sentence by one byte, which swaps
     *    the byte order of every sample after it - broadband noise.
     */
    static uint8_t carry = 0;
    static bool    has_carry = false;

    const uint8_t *p = (const uint8_t *)pcm16;
    size_t left_bytes = len;

    /* Finish a sample split across the previous chunk. */
    if (has_carry) {
        const uint8_t pair[2] = { carry, p[0] };
        int16_t first;
        memcpy(&first, pair, sizeof(first));

        int16_t frame[2] = { first, 0 };
        size_t written = 0;
        esp_err_t err = i2s_channel_write(s_tx, frame, sizeof(frame), &written,
                                          pdMS_TO_TICKS(2000));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2s write failed: %s", esp_err_to_name(err));
            return err;
        }
        has_carry = false;
        p++;
        left_bytes--;
        if (left_bytes == 0) {
            return ESP_OK;
        }
    }

    const size_t whole_samples = left_bytes / 2;
    if ((left_bytes & 1) != 0) {
        carry = p[left_bytes - 1];
        has_carry = true;
    }

    const int16_t *src = (const int16_t *)p;
    int16_t block[240 * 2];
    size_t done = 0;
    while (done < whole_samples) {
        size_t n = whole_samples - done;
        if (n > 240) {
            n = 240;
        }
        size_t o = 0;
        for (size_t i = 0; i < n; i++) {
            block[o++] = src[done + i];   /* left  */
            block[o++] = 0;               /* right */
        }

        size_t written = 0;
        esp_err_t err = i2s_channel_write(s_tx, block, o * sizeof(int16_t),
                                          &written, pdMS_TO_TICKS(2000));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2s write failed: %s", esp_err_to_name(err));
            return err;
        }
        if (written == 0) {
            return ESP_ERR_TIMEOUT;
        }
        done += written / (2 * sizeof(int16_t));
    }

    return ESP_OK;
}

esp_err_t audio_output_stream_write(const void *pcm16, size_t len)
{
    if (s_tx == NULL || pcm16 == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* No pre-roll configured, or already playing: straight through. */
    if (s_playing || s_pre_cap == 0) {
        return raw_write(pcm16, len);
    }

    const uint8_t *p = (const uint8_t *)pcm16;
    size_t off = 0;

    while (off < len) {
        size_t room = s_pre_cap - s_pre_len;
        size_t n = len - off;
        if (n > room) {
            n = room;
        }
        memcpy(s_pre + s_pre_len, p + off, n);
        s_pre_len += n;
        off += n;

        if (s_pre_len < s_pre_cap) {
            continue;
        }

        /* Bank is full - start playing and stay in pass-through from here. */
        s_playing = true;
        esp_err_t err = audio_output_set_enabled(true);
        if (err != ESP_OK) {
            return err;
        }
        ESP_LOGI(TAG, "pre-roll filled (%u ms) - playback starts",
                 (unsigned)(CONFIG_AUDIO_OUT_PREROLL_MS));

        err = raw_write(s_pre, s_pre_len);
        s_pre_len = 0;
        if (err != ESP_OK) {
            return err;
        }
        if (off < len) {
            return raw_write(p + off, len - off);
        }
        return ESP_OK;
    }

    return ESP_OK;
}

esp_err_t audio_output_stream_end(void)
{
    if (s_tx == NULL) {
        return ESP_OK;
    }

    /* A sentence shorter than the pre-roll never triggered playback. */
    if (!s_playing && s_pre_len > 0) {
        s_playing = true;
        esp_err_t err = audio_output_set_enabled(true);
        if (err == ESP_OK) {
            err = raw_write(s_pre, s_pre_len);
        }
        s_pre_len = 0;
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "flush of the pre-roll failed: %s", esp_err_to_name(err));
        }
    }

    /*
     * Let the FIFO drain before muting. The DMA is 682 ms deep, so the old
     * 150 ms cut the tail of every sentence off once streaming was introduced.
     * One wait per reply, not per sentence.
     */
    vTaskDelay(pdMS_TO_TICKS(800));
    return audio_output_set_enabled(false);
}

/* Adapter so the TTS client can push blocks straight into the amplifier. */
esp_err_t audio_output_sink(const void *pcm, size_t len, void *user)
{
    (void)user;

    /*
     * Per-block timing. A blocking i2s write should take about as long as the
     * audio it carries once the DMA is saturated, so any block that takes much
     * longer than len/4 ms means the delay is upstream (network) rather than in
     * the amplifier - which is the difference between "lower buffer" and
     * "server is too slow".
     */
    const int64_t t0 = esp_timer_get_time();
    esp_err_t err = audio_output_stream_write(pcm, len);
    const int64_t dt = (esp_timer_get_time() - t0) / 1000;

    if (s_t_first == 0) {
        s_t_first = t0;
    }

    ESP_LOGI(TAG, "block %4u B  took %5lld ms  (audio %4.0f ms)  at %5lld ms",
             (unsigned)len, (long long)dt,
             (double)len / 2.0 / app_cfg_audio_out_sample_rate() * 1000.0,
             (long long)((t0 - s_t_first) / 1000));

    return err;
}

esp_err_t audio_output_play(const void *pcm, size_t len)
{
    if (pcm == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = audio_output_stream_begin();
    if (err == ESP_OK) {
        err = audio_output_stream_write(pcm, len);
    }
    esp_err_t end_err = audio_output_stream_end();

    return err != ESP_OK ? err : end_err;
}
