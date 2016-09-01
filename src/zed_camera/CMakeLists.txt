SET(execName zed_camera_sensor)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

SET(CUDA_VERSION "7.5")
SET(OCV_VERSION "3.1")


find_package(ZED 1.0 REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(OpenCV ${OCV_VERSION} COMPONENTS core highgui imgproc ${OCV_MODULE} REQUIRED)
find_package(CUDA ${CUDA_VERSION} REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
#CARMEN INCLUDE DIR
include_directories($ENV{CARMEN_HOME}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
#CARMEN LIBRARY DIR
link_directories($ENV{CARMEN_HOME}/lib)

add_definitions(-std=c++0x)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3" )

#EXECUTAVEIS
ADD_EXECUTABLE(${execName} zed_camera_sensor_main.cpp)
ADD_EXECUTABLE(zed_camera_sensor_viewer zed_camera_sensor_viewer.cpp)

#LIB
ADD_LIBRARY(zed_camera_sensor_interface zed_camera_sensor_interface.c)

TARGET_LINK_LIBRARIES(${execName}
        ${ZED_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${CUDA_LIBRARIES} ${CUDA_nppi_LIBRARY} ${CUDA_npps_LIBRARY}
        global z ipc param_interface zed_camera_sensor_interface
        )

TARGET_LINK_LIBRARIES(zed_camera_sensor_viewer
        ${OpenCV_LIBRARIES}
        global ipc zed_camera_sensor_interface
        )

TARGET_LINK_LIBRARIES(zed_camera_sensor_interface
        global z ipc
        )