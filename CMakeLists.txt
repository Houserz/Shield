# Minimum required CMake version
cmake_minimum_required(VERSION 3.16)

# Set path for extra components
set(EXTRA_COMPONENT_DIRS components)

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Project definition
project(shield_daq_pipeline)
