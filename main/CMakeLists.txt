idf_component_register(
    SRCS "main_dual_core.c"
    INCLUDE_DIRS "."
    REQUIRES sensor_hal drivers data_types sd_storage
)
