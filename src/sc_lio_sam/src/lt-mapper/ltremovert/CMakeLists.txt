cmake_minimum_required(VERSION 2.8.3)
project(removert)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17) # C++17 to use stdc++fs
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  image_transport
)

find_package(OpenMP REQUIRED)
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl" CACHE PATH "PCL cmake dir" FORCE)
# Ubuntu 26.04: o VTK 9.5 do apt define VTK::jsoncpp apontando para JsonCpp::JsonCpp,
# mas nao chama find_package(JsonCpp) quando o PCL o carrega -> "target was not found"
# no generate. Carregar o jsoncpp antes cria o alvo. Ver ~/ros_src/NOTAS.md.
find_package(jsoncpp QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
# find_package(GTSAM REQUIRED QUIET)

# add_message_files(
#   DIRECTORY msg
#   FILES
#   # cloud_info.msg
# )

# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   std_msgs
#   nav_msgs
#   sensor_msgs
# )

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL 
  # GTSAM

  CATKIN_DEPENDS 
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime 
  message_generation
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	# ${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  # ${GTSAM_LIBRARY_DIRS}
)

###########
## Build ##
###########

# Range Image Projection
add_executable( ${PROJECT_NAME}_removert src/removert_main.cpp
  src/utility.cpp 
  src/RosParamServer.cpp
  src/Session.cpp
  src/Removerter.cpp
)

add_executable(${PROJECT_NAME}_reconstruct_feature_map src/reconstruct_feature_map.cpp)

add_dependencies(${PROJECT_NAME}_reconstruct_feature_map ${catkin_EXPORTED_TARGETS})
target_compile_options(${PROJECT_NAME}_reconstruct_feature_map PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_reconstruct_feature_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  stdc++fs
)

add_dependencies(${PROJECT_NAME}_removert ${catkin_EXPORTED_TARGETS})
target_compile_options(${PROJECT_NAME}_removert PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_removert 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  stdc++fs
)


