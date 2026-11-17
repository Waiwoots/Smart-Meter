idf_component_register(
    SRCS "main.c"
    EMBED_FILES "index.html" "wifi_setup.html"
    INCLUDE_DIRS "."
    REQUIRES
        ble_transport
        dlms
        driver
        esp_driver_spi
        esp_lcd
        esp_lcd_ili9488
        led_strip
        esp_wifi
        esp_event
        esp_netif
        esp_http_server
        nvs_flash
        meter_mqtt
)