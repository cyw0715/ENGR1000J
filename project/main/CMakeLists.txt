idf_component_register(
    SRCS "main.c" "camera.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_cam_sensor esp_sccb_intf esp_timer
)
