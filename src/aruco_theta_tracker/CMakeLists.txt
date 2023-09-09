cmake_minimum_required(VERSION 2.8)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall")

find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)
find_package(jsoncpp REQUIRED)


# astro
include_directories($ENV{CARMEN_HOME}/include)
link_directories($ENV{CARMEN_HOME}/lib)

# tf
include_directories($ENV{CARMEN_HOME}/sharedlib/libtf/src)
include_directories(/usr/include/bullet/)
include_directories(/usr/local/include/bullet/)

add_executable(aruco_theta_tracker aruco_theta_tracker_main.cpp PoseTracker.hpp aruco_theta_tracker.hpp aruco_theta_tracker.cpp)
add_executable(create_board create_board.cpp)

target_link_libraries(aruco_theta_tracker aruco opencv_highgui opencv_calib3d jsoncpp_lib 
                      global ipc param_interface camera_drivers bumblebee_basic_interface localize_ackerman_interface 
                      tf boost_thread-mt boost_signals boost_system)
target_link_libraries(create_board aruco opencv_highgui opencv_imgproc)
