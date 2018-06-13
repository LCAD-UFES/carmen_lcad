cmake_minimum_required(VERSION 2.8)
project(rpi_camera)

option(ENABLE_GUI "Enable Graphic Interface" ON)

if (ENABLE_GUI)
	add_definitions(-DENABLE_GUI)
endif()

set(CMAKE_CXX_FLAGS	 "${CMAKE_CXX_FLAGS} -O3")

find_package(OpenCV REQUIRED)
set(raspicam_DIR "/usr/local/lib/cmake")
find_package(raspicam REQUIRED)
include_directories (${OpenCV_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES pi_camera_server_driver.cpp)
add_executable(pi_camera_server_driver ${SOURCE_FILES})
target_link_libraries(pi_camera_server_driver ${OpenCV_LIBRARIES} ${raspicam_CV_LIBS})

set(SOURCE_FILES pi_camera_test.cpp)
add_executable(pi_camera_test ${SOURCE_FILES})
target_link_libraries(pi_camera_server_driver ${OpenCV_LIBRARIES} ${raspicam_CV_LIBS})
