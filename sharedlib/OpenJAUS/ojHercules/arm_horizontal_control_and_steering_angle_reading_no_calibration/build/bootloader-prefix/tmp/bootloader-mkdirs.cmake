# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/hercules/carmen_lcad/sharedlib/OpenJAUS/ojHercules/esp-idf-arduino/esp-idf/components/bootloader/subproject"
  "/home/hercules/carmen_lcad/sharedlib/OpenJAUS/ojHercules/arm_horizontal_control_and_steering_angle_reading_no_calibration/build/bootloader"
  "/home/hercules/carmen_lcad/sharedlib/OpenJAUS/ojHercules/arm_horizontal_control_and_steering_angle_reading_no_calibration/build/bootloader-prefix"
  "/home/hercules/carmen_lcad/sharedlib/OpenJAUS/ojHercules/arm_horizontal_control_and_steering_angle_reading_no_calibration/build/bootloader-prefix/tmp"
  "/home/hercules/carmen_lcad/sharedlib/OpenJAUS/ojHercules/arm_horizontal_control_and_steering_angle_reading_no_calibration/build/bootloader-prefix/src/bootloader-stamp"
  "/home/hercules/carmen_lcad/sharedlib/OpenJAUS/ojHercules/arm_horizontal_control_and_steering_angle_reading_no_calibration/build/bootloader-prefix/src"
  "/home/hercules/carmen_lcad/sharedlib/OpenJAUS/ojHercules/arm_horizontal_control_and_steering_angle_reading_no_calibration/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/hercules/carmen_lcad/sharedlib/OpenJAUS/ojHercules/arm_horizontal_control_and_steering_angle_reading_no_calibration/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/hercules/carmen_lcad/sharedlib/OpenJAUS/ojHercules/arm_horizontal_control_and_steering_angle_reading_no_calibration/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
