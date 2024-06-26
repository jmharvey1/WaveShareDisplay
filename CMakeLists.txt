cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(WaveShareDisplay)
#get_filename_component(configName "${CMAKE_BINARY_DIR}" NAME)
#list(APPEND kconfigs "${CMAKE_SOURCE_DIR}/.pio/libdeps/${configName}/lvgl/Kconfig")
list(APPEND kconfigs "/home/jim/Documents/PlatformIO/Projects/WaveShareDisplay/.pio/libdeps/esp32-s3-devkitc-1/lvgl/Kconfig")

