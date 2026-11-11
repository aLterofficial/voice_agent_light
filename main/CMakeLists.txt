idf_component_register(
    SRCS
        "main.c"
        "app_config.c"
        "audio_input.c"
        "audio_output.c"
        "wifi_mgr.c"
        "console_cmd.c"
        "power_save.c"
        "radio_bringup.c"
        "rail_monitor.c"
        "servo.c"
        "voice_cmd.c"
        "i2c_scan.c"
        "iflytek_stt.c"
        "iflytek_tts.c"
        "siliconflow_stt.c"
        "siliconflow_tts.c"
        "speech_backend.c"
        "deepseek_client.c"
        "wake_word.c"
        "led_status.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_wifi
        nvs_flash
        esp_http_client
        esp_websocket_client
        cjson
        driver
        esp_driver_i2s
        esp_driver_gpio
        esp_driver_uart
        esp_driver_i2c   # rail monitor's OLED
        esp_adc          # rail monitor's rail sampling
        esp_driver_ledc  # servo PWM
        led_strip
        mbedtls
        esp_timer
        # header homes that are not pulled in transitively:
        esp_app_format   # esp_app_desc.h
        esp_psram        # esp_psram.h
        heap             # esp_heap_caps.h
)
