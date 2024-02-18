# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/breno/esp/esp-idf/components/bootloader/subproject"
  "/home/breno/carmen_lcad/sharedlib/OpenJAUS/ojHercules/hercules-esp-breno/build/bootloader"
  "/home/breno/carmen_lcad/sharedlib/OpenJAUS/ojHercules/hercules-esp-breno/build/bootloader-prefix"
  "/home/breno/carmen_lcad/sharedlib/OpenJAUS/ojHercules/hercules-esp-breno/build/bootloader-prefix/tmp"
  "/home/breno/carmen_lcad/sharedlib/OpenJAUS/ojHercules/hercules-esp-breno/build/bootloader-prefix/src/bootloader-stamp"
  "/home/breno/carmen_lcad/sharedlib/OpenJAUS/ojHercules/hercules-esp-breno/build/bootloader-prefix/src"
  "/home/breno/carmen_lcad/sharedlib/OpenJAUS/ojHercules/hercules-esp-breno/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/breno/carmen_lcad/sharedlib/OpenJAUS/ojHercules/hercules-esp-breno/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/breno/carmen_lcad/sharedlib/OpenJAUS/ojHercules/hercules-esp-breno/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
