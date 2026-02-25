idf_component_register(
    SRCS "main_dual_core.cpp"
    INCLUDE_DIRS "."
    REQUIRES sensor_hal drivers data_types sd_storage esp32_BNO08x
)
