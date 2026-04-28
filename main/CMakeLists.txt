idf_component_register(SRCS "max17043.c" "main.c" 
                       INCLUDE_DIRS "."
                       REQUIRES espressif__led_strip mdns esp_https_ota mqtt esp_http_client driver esp_driver_uart led_strip esp_timer esp_event esp_netif esp_wifi nvs_flash mqtt)
