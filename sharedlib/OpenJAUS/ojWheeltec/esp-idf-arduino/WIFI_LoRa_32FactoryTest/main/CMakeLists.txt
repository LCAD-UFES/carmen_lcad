# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

string(APPEND CMAKE_CXX_FLAGS " -Wno-unused-but-set-parameter")

list(APPEND COMPONENT_INCLUDES "Heltec_ESP32/src" "Heltec_ESP32/src/lora" "Heltec_ESP32/src/oled")

list(APPEND COMPONENT_SRCS 
             "Heltec_ESP32/src/BMP180.cpp"
             "Heltec_ESP32/src/heltec.cpp"
             "Heltec_ESP32/src/lora/LoRa.cpp"
             "Heltec_ESP32/src/oled/OLEDDisplay.cpp"
             "Heltec_ESP32/src/oled/OLEDDisplayUi.cpp"
             )

list(APPEND COMPONENT_SRCS "WiFi_LoRa_32FactoryTest.cpp")

register_component()
