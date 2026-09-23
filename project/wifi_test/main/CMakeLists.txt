idf_component_register(
    SRCS "main.c" "camera.c" "imu.c" "mega_sr04.c" "actuator.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_wifi nvs_flash esp_event esp_netif esp_timer esp_cam_sensor esp_sccb_intf driver esp_driver_jpeg esp_http_server usb
)
