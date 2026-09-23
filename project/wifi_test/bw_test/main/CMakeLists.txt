idf_component_register(
    SRCS "bw_main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_wifi nvs_flash esp_event esp_netif esp_timer lwip
)
