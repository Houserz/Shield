# Review-1: stamp the firmware git hash into the build so meta.json carries it
execute_process(
    COMMAND git describe --always --dirty --abbrev=12
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE FW_GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT FW_GIT_HASH)
    set(FW_GIT_HASH "unknown")
endif()

idf_component_register(
    SRCS "main_dual_core.cpp"
    INCLUDE_DIRS "."
    REQUIRES sensor_hal drivers data_types sd_storage esp32_BNO08x driver esp_timer
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE FW_GIT_HASH="${FW_GIT_HASH}")
