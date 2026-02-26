idf_component_register(
    SRCS "driver_gyroscope.cpp" "driver_magnetometer.cpp" "driver_accelerometer.cpp" "driver_photodiode.c" "driver_inmp441.c" "driver_bno085.c"
         "driver_vibration.c"
         "driver_current.c"
         "driver_mpl3115.c"
         "driver_mcp9808.c"
         "driver_ads1115.c"
    INCLUDE_DIRS "."
    REQUIRES sensor_hal driver esp_driver_i2s esp_adc esp32_BNO08x
)

