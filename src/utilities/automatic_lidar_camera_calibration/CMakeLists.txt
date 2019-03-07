project (mi-extrinsic-calib)

# CMAKE TWEAKS
#========================================================================
cmake_minimum_required (VERSION 2.6)

#These variables will be inherited in child directories
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
set (SRC_DIR "${CMAKE_SOURCE_DIR}/src")

# CHECK THE DEPENDENCIES
# GLIB
find_package (Glib REQUIRED)
# GSL
find_package (GSL REQUIRED)
# Opencv
find_package (OpenCV REQUIRED)

set (EXECUTABLE_OUTPUT_PATH 
  "${PROJECT_SOURCE_DIR}/bin"
  CACHE PATH
  "Single directory for all executables."
  )

set (LIBRARY_OUTPUT_PATH 
  "${PROJECT_SOURCE_DIR}/lib"
  CACHE PATH
  "Single directory for all executables."
  )

# COMPILE THE SOURCE
#========================================================================
add_subdirectory (src)
