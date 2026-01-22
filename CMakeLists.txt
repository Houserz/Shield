# 最小CMake版本要求
cmake_minimum_required(VERSION 3.16)

# 设置组件路径
set(EXTRA_COMPONENT_DIRS components)

# 包含ESP-IDF构建系统
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# 项目定义
project(shield_daq_pipeline)
