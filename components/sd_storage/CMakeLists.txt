idf_component_register(
    SRCS "sd_storage.c"
    INCLUDE_DIRS "include"
    REQUIRES data_types fatfs sdmmc driver
)

