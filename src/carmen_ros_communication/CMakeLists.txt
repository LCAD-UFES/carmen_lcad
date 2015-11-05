cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_add_executable(carmen_ros_communication src/carmen_ros_communication_main.cpp)
rosbuild_add_executable(bumblebee_publisher src/bumblebee_publisher.cpp src/carmen_src/ipc_wrapper.c)

#IPC
set(IPC_INC_DIR $ENV{CARMEN_HOME}/include/carmen/)
set(CARMEN_INC_DIR $ENV{CARMEN_HOME}/include)
set(IPC_LIB_DIR $ENV{CARMEN_HOME}/lib/)
include_directories(${IPC_INC_DIR}) 
include_directories(${CARMEN_INC_DIR})
#target_link_libraries(carmen_ros_communication ${IPC_LIB_DIR}/libipc.a)
target_link_libraries(bumblebee_publisher z)
target_link_libraries(bumblebee_publisher ${IPC_LIB_DIR}/libipc.a)
target_link_libraries(bumblebee_publisher ${IPC_LIB_DIR}/libcarmenserial.a)
target_link_libraries(bumblebee_publisher ${IPC_LIB_DIR}/libglobal.a)

#boost deps
rosbuild_add_boost_directories()
#rosbuild_link_boost(carmen_ros_communication signals)
rosbuild_link_boost(bumblebee_publisher signals)
