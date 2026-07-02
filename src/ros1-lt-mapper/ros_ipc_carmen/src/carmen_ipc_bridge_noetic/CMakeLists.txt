cmake_minimum_required(VERSION 3.0.2)
project(carmen_ipc_bridge)

# ─── C++ Standard ────────────────────────────────────────────────────────────
add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# ─── ROS1 / catkin ───────────────────────────────────────────────────────────
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ─── CARMEN / IPC ────────────────────────────────────────────────────────────
if(NOT DEFINED ENV{CARMEN_HOME})
  message(FATAL_ERROR "CARMEN_HOME não está definida.")
endif()

set(CARMEN_HOME $ENV{CARMEN_HOME})

set(CARMEN_INCLUDE_DIRS
  ${CARMEN_HOME}/include
  ${CARMEN_HOME}/sharedlib/libtf/src
  ${CARMEN_HOME}/sharedlib/prob_models
  /usr/local/include/bullet
  /usr/include/bullet
  /usr/local/carmen_boost/include
)

set(CARMEN_LIBRARY_DIRS
  ${CARMEN_HOME}/lib
  ${CARMEN_HOME}/sharedlib/libtf/src
  ${CARMEN_HOME}/sharedlib/prob_models
  /usr/local/carmen_boost/lib
)

# ─── CARMEN_LIBRARIES ────────────────────────────────────────────────────────
# Adicionamos velodyne_interface e xsens_interface de volta para o bridge funcionar!
set(CARMEN_LIBRARIES
  -Wl,--start-group
  ipc global param_interface
  velodyne_interface xsens_interface
  map_interface map_util map_io
  prob_models grid_mapping
  tf BulletCollision BulletDynamics BulletSoftBody LinearMath boost_signals boost_system
  -Wl,--end-group
)

find_package(Boost REQUIRED COMPONENTS thread)

# ─── catkin_package ──────────────────────────────────────────────────────────
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ─── ipc_bridge_node ─────────────────────────────────────────────────────────
add_executable(ipc_bridge_node src/ipc_bridge_node.cpp)
add_dependencies(ipc_bridge_node ${catkin_EXPORTED_TARGETS})
target_include_directories(ipc_bridge_node PRIVATE ${CARMEN_INCLUDE_DIRS})
target_link_directories(ipc_bridge_node PRIVATE ${CARMEN_LIBRARY_DIRS})
target_link_libraries(ipc_bridge_node
  ${catkin_LIBRARIES}
  Boost::thread
  ${CARMEN_LIBRARIES}
)

# ─── pointcloud_node (sem dependência do CARMEN, só ROS) ────────────────────
add_executable(pointcloud_node src/pointcloud_node.cpp)
add_dependencies(pointcloud_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_node ${catkin_LIBRARIES})

# ─── Install ──────────────────────────────────────────────────────────────
install(TARGETS ipc_bridge_node pointcloud_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

set_target_properties(ipc_bridge_node PROPERTIES
  BUILD_RPATH ""
  INSTALL_RPATH ""
  SKIP_BUILD_RPATH FALSE
  BUILD_WITH_INSTALL_RPATH FALSE
)