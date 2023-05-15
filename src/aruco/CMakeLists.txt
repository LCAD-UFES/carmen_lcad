cmake_minimum_required(VERSION 2.8)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    libavformat
    libavcodec
    libswscale
    libavutil
)

# carmen
include_directories($ENV{CARMEN_HOME}/include)
link_directories($ENV{CARMEN_HOME}/lib)

# tf
include_directories($ENV{CARMEN_HOME}/sharedlib/libtf/src)
include_directories(/usr/include/bullet/)
include_directories(/usr/local/include/bullet/)

include_directories($ENV{CARMEN_HOME}/src/localize_ackerman)
link_directories($ENV{CARMEN_HOME}/src/localize_ackerman)

add_executable(aruco-bin aruco_main.cpp aruco_interface.cpp aruco_interface.h aruco_messages.h)
add_executable(aruco_test aruco_test.cpp aruco_interface.cpp aruco_interface.h aruco_messages.h)
add_executable(create_board create_board.cpp)
#add_executable(aruco_vs_velodyne aruco_vs_velodyne.cpp aruco_interface.cpp aruco_interface.h aruco_messages.h)
#add_executable(localize_ackerman_interface.c localize_ackerman_interface.h localize_ackerman_messages.h)

# link_directories($ENV{CARMEN_HOME}/src/camera_drivers)

target_link_libraries(aruco-bin aruco opencv_highgui opencv_calib3d 
                      global ipc param_interface camera_drivers bumblebee_basic_interface localize_ackerman_interface  
                      tf boost_thread-mt boost_signals boost_system)
target_link_libraries(aruco_test global ipc param_interface)
target_link_libraries(create_board aruco opencv_highgui)
#target_link_libraries(aruco_vs_velodyne aruco localize_ackerman_interface)

set_target_properties(aruco-bin PROPERTIES OUTPUT_NAME aruco)
