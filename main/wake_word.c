/*
 * Wake word detection.
 *
 * Two backends, selected by Kconfig:
 *
 *   VAD      - pure energy threshold. No extra components, no model files.
 *              Triggers on any loud sound, so it fires on coughs, doors and
 *              speech that is not directed at the device. Good enough to
 *              bring the hardware up and to test the rest of the pipeline.
 *
 *   WAKENET  - ESP-SR WakeNet, an offline neural wake word detector. This is
 *              the real thing: it only fires on the configured phrase, for
 *              example "你好小鑫". Requires the esp-sr managed component and
 *              a model partition in the partition table.
 *
 * NOTE on audio quality: WakeNet needs a clean 16 kHz mono stream. That is
 * exactly why this project uses an I2S digital microphone (INMP441) rather
 * than an analogue one sampled through the ADC - the ADC path on this chip
 * is noisy and is disturbed by the WiFi radio, which destroys recognition
 * accuracy.
 */

#include <string.h>

#include "app_internal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if CONFIG_WAKE_BACKEND_WAKENET
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "model_path.h"
#endif

static const char *TAG = "wake";

#if CONFIG_WAKE_BACKEND_WAKENET
static const esp_afe_sr_iface_t *s_afe = NULL;
static esp_afe_sr_data_t *s_afe_data = NULL;
static volatile bool s_wake_hit = false;

static void afe_task(void *arg)
{
    while (1) {
        afe_fetch_result_t *res = s_afe->fetch_with_delay(s_afe_data, portMAX_DELAY);
        if (res == NULL || res->ret_value == ESP_FAIL) {
            continue;
        }
        if (res->wakeup_state == WAKENET_DETECTED) {
            ESP_LOGI(TAG, "wake word detected");
            s_wake_hit = true;
        }
    }
}
#endif

esp_err_t wake_word_init(void)
{
#if CONFIG_WAKE_BACKEND_WAKENET
    srmodel_list_t *models = esp_srmodel_init("model");
    if (models == NULL) {
        ESP_LOGE(TAG, "no ESP-SR models found - is the 'model' partition "
                      "present in the partition table?");
        return ESP_ERR_NOT_FOUND;
    }

    char *wn_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);
    if (wn_name == NULL) {
        ESP_LOGE(TAG, "no WakeNet model found in the model partition");
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "using WakeNet model: %s", wn_name);

    afe_config_t *cfg = afe_config_init("M", models, AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
    if (cfg == NULL) {
        return ESP_FAIL;
    }
    /* Single microphone: no reference channel, no playback reference. */
    cfg->aec_init = false;
    cfg->se_init = true;
    cfg->vad_init = false;
    cfg->wakenet_init = true;
    cfg->wakenet_mode = DET_MODE_90;
    cfg->agc_init = true;

    s_afe = esp_afe_handle_from_config(cfg);
    s_afe_data = s_afe->create_from_config(cfg);
    afe_config_free(cfg);

    xTaskCreate(afe_task, "afe", 8 * 1024, NULL, 5, NULL);
    ESP_LOGI(TAG, "WakeNet ready");
    return ESP_OK;
#else
    ESP_LOGI(TAG, "energy-VAD wake backend (threshold rms %d)", CONFIG_WAKE_TRIGGER_RMS);
    ESP_LOGW(TAG, "the VAD backend fires on ANY loud sound. Enable "
                  "CONFIG_WAKE_BACKEND_WAKENET for real wake word detection.");
    return ESP_OK;
#endif
}

esp_err_t wake_word_wait(int timeout_ms)
{
#if CONFIG_WAKE_BACKEND_WAKENET
    s_wake_hit = false;
    const int slice = 50;
    int waited = 0;

    while (timeout_ms == 0 || waited < timeout_ms) {
        if (s_wake_hit) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(slice));
        waited += slice;
    }
    return ESP_ERR_TIMEOUT;
#else
    return audio_input_wait_for_sound(CONFIG_WAKE_TRIGGER_RMS, timeout_ms);
#endif
}
