# Top-level toggles for the streamer component.
# Override from the project root with e.g.:
#     idf.py -DSTREAMER_USB=1 -DSTREAMER_WIFI=1 build
# Default: USB on, Wi-Fi off (Wi-Fi radio adds ~100 mA average draw).
if(NOT DEFINED STREAMER_USB)
    set(STREAMER_USB 1 CACHE STRING "Enable USB streaming (1/0)")
endif()
if(NOT DEFINED STREAMER_WIFI)
    set(STREAMER_WIFI 0 CACHE STRING "Enable Wi-Fi streaming (1/0)")
endif()

# Conditional REQUIRES so unused stacks aren't linked in.
set(_streamer_reqs data_types log)
if(STREAMER_USB)
    list(APPEND _streamer_reqs driver)
endif()
if(STREAMER_WIFI)
    list(APPEND _streamer_reqs esp_wifi esp_netif esp_event nvs_flash lwip)
endif()

idf_component_register(
    SRCS "streamer.c"
    INCLUDE_DIRS "include"
    REQUIRES ${_streamer_reqs}
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    STREAMER_ENABLE_USB=${STREAMER_USB}
    STREAMER_ENABLE_WIFI=${STREAMER_WIFI}
)

message(STATUS "streamer: USB=${STREAMER_USB}  WIFI=${STREAMER_WIFI}")
