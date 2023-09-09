# DJ's Arduino as an ESP-IDF component template project
 - This template project has the target set to 'esp32' ... You will need to adjust the sdkconfig if you are using a different chip (esp32s2, esp32s3, esp32c3, esp32h2).
 - See [https://therealdj.github.io/esp32](https://therealdj.github.io/esp32) for install instructions

## sdkconfig changes made (compared to IDF default)
 - Changed partition table to support OTA functionality (PARTITION_TABLE_TWO_OTA & PARTITION_TABLE_FILENAME)
 - Changed CPU frequency to 240MHz (ESP32_DEFAULT_CPU_FREQ_240 & ESP32_DEFAULT_CPU_FREQ_MHZ)
 - Enabled mbedTls PSK (MBEDTLS_PSK_MODES) and key exchange ciphersuites (MBEDTLS_KEY_EXCHANGE_*)
 - Changed RTOS tick rate to 1000 Hz (FREERTOS_HZ)
 - Enabled Arduino autostart - call setup() and loop() as an Arduino project does (AUTOSTART_ARDUINO)

## cmake additions
### CMakeLists.txt
 - esp32.rom.redefined.ld was not being linked
 - Added ```list(APPEND CMAKE_EXE_LINKER_FLAGS "-T esp32.rom.redefined.ld")```

### main/CMakeLists.txt
 - When compiling code with a function that updates a pointer, I was getting an error due to gcc default ```-Werror=unused-but-set-parameter```
 - Added ```string(APPEND CMAKE_CXX_FLAGS " -Wno-unused-but-set-parameter")```
