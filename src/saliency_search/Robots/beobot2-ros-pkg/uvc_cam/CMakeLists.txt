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

#add_definitions("-D_FILE_OFFSET_BITS=64") #No idea what this is for... maybe something for the SDL library?

# auto-generated dynamic reconfiguration GUI
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#Build the uvc_cam library
rosbuild_add_library(uvc_cam src/uvc_cam/uvc_cam.cpp)

# librt is needed for clock_gettime();  not sure how this interacts with
# real-time....
#target_link_libraries(dc1394cam rt)

#rosbuild_add_executable(view src/view/view.cpp)
#target_link_libraries(view uvc_cam)
#target_link_libraries(view ${SDL_LIBRARIES})

#rosbuild_add_executable(enumerate src/enumerate/enumerate.cpp)
#target_link_libraries(enumerate uvc_cam)

#rosbuild_add_executable(dump src/dump/dump.cpp src/dump/avilib.c)
#target_link_libraries(dump uvc_cam)

#rosbuild_add_executable(play src/play.cpp)
#target_link_libraries(play uvc_cam)

#rosbuild_add_executable(sender src/sender.cpp)
#target_link_libraries(sender uvc_cam)

rosbuild_add_executable(uvc_cam_node src/uvc_cam_node.cpp)
target_link_libraries(uvc_cam_node uvc_cam)

#rosbuild_add_executable(senderStereo src/stereo_sender.cpp)
#target_link_libraries(senderStereo uvc_cam)

#rosbuild_add_executable(dump_send src/dump/dump_send.cpp src/dump/avilib.c)
#target_link_libraries(dump_send uvc_cam)
