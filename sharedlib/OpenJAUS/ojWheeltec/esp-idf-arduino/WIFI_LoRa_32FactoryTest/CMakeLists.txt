# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

#add_definitions(-DESP32) 

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

include_directories("main/Heltec_ESP32/src" "main/Heltec_ESP32/src/lora" "main/Heltec_ESP32/src/oled" "main/WiFi_Kit_series/esp32/variants/wifi_lora_32_V2")

project(WIFI_LoRa_32FactoryTest)

list(APPEND CMAKE_EXE_LINKER_FLAGS "-T esp32.rom.redefined.ld")
