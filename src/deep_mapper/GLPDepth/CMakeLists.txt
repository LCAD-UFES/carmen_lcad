cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(custom_ops)

find_package(Torch REQUIRED)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBS})

add_executable(libglpdepth libglpdepth.cpp)
target_link_libraries(libglpdepth ${OpenCV_LIBS} "${TORCH_LIBRARIES}")
set_property(TARGET libglpdepth PROPERTY CXX_STANDARD 14)