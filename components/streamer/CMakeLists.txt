# Transport switches live in include/streamer_config.h.
# Keep both dependency sets available so changing that header only needs a
# normal idf.py build, not a CMake -D flag or fullclean.

idf_component_register(
    SRCS "streamer.c"
    INCLUDE_DIRS "include"
    REQUIRES data_types
    PRIV_REQUIRES
        log
        driver
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        lwip
        esp_hw_support
)

message(STATUS "streamer: transport switches configured in components/streamer/include/streamer_config.h")
