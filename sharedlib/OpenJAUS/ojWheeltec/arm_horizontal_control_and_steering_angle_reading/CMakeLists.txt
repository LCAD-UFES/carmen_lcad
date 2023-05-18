# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(arm_horizontal_control_and_steering_angle_reading)

list(APPEND CMAKE_EXE_LINKER_FLAGS "-T esp32.rom.redefined.ld")
