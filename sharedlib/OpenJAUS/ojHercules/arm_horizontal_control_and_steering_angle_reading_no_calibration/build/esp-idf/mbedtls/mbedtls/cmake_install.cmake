# Install script for directory: /home/hercules/carmen_lcad/sharedlib/OpenJAUS/ojHercules/esp-idf-arduino/esp-idf/components/mbedtls/mbedtls

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/home/hercules/.espressif/tools/xtensa-esp32-elf/esp-2021r2-8.4.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/hercules/carmen_lcad/sharedlib/OpenJAUS/ojHercules/arm_horizontal_control_and_steering_angle_reading_no_calibration/build/esp-idf/mbedtls/mbedtls/library/cmake_install.cmake")
  include("/home/hercules/carmen_lcad/sharedlib/OpenJAUS/ojHercules/arm_horizontal_control_and_steering_angle_reading_no_calibration/build/esp-idf/mbedtls/mbedtls/include/cmake_install.cmake")

endif()

