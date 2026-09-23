idf_component_register(
    SRCS "main.c" "camera.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES nvs_flash esp_timer esp_cam_sensor esp_sccb_intf driver esp_driver_jpeg
)
