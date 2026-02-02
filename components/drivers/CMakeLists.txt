idf_component_register(
    SRCS "driver_photodiode.c" "driver_inmp441.c" "driver_bno085.c"
         "driver_vibration.c"
         "driver_current.c"
         "driver_mpl3115.c"
         "driver_mcp9808.c"
    INCLUDE_DIRS "."
    REQUIRES sensor_hal
)

