idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS ""
                       REQUIRES esp_wifi esp_http_client esp_netif esp_event nvs_flash console json esp_system
                       PRIV_REQUIRES esp_driver_uart)
