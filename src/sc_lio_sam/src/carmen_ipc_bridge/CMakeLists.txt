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
  visualization_msgs
  rosgraph_msgs
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
)

set(CARMEN_LIBRARY_DIRS
  ${CARMEN_HOME}/lib
  ${CARMEN_HOME}/sharedlib/libtf/src
  ${CARMEN_HOME}/sharedlib/prob_models
)

# ─── CARMEN_LIBRARIES ────────────────────────────────────────────────────────
# Adicionamos velodyne_interface e xsens_interface de volta para o bridge funcionar!
# imu_interface adicionado pro fallback xsens -> IMU do lidar (carmen_imu_message)
set(CARMEN_LIBRARIES
  -Wl,--start-group
  ipc global param_interface
  velodyne_interface xsens_interface imu_interface base_ackerman_interface
  gps_xyz_interface gps_nmea_interface fused_odometry_interface
  collision_detection localize_ackerman_interface
  map_interface map_util map_io
  prob_models grid_mapping
  tf BulletCollision BulletDynamics BulletSoftBody LinearMath
  -Wl,--end-group
  # libmap_io do CARMEN grava/le mapa comprimido (gz*/inflate/deflate) e nao
  # arrasta a zlib sozinha -- sem isto o link dos 4 nos quebra em
  # "undefined reference to `gzopen'".
  z
)

find_package(Boost REQUIRED COMPONENTS thread filesystem)

# ─── catkin_package ──────────────────────────────────────────────────────────
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs rosgraph_msgs tf2 tf2_ros tf2_geometry_msgs
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

# ─── pointcloud_node (lê range_division_factor do param_daemon do CARMEN) ────
add_executable(pointcloud_node src/pointcloud_node.cpp)
add_dependencies(pointcloud_node ${catkin_EXPORTED_TARGETS})
target_include_directories(pointcloud_node PRIVATE ${CARMEN_INCLUDE_DIRS})
target_link_directories(pointcloud_node PRIVATE ${CARMEN_LIBRARY_DIRS})
target_link_libraries(pointcloud_node
  ${catkin_LIBRARIES}
  Boost::thread
  ${CARMEN_LIBRARIES}
)

# ─── gps_localization_node (fusão/correção de GPS -> /odometry/gps p/ o SC-LIO-SAM) ──
add_executable(gps_localization_node src/gps_localization_node.cpp)
add_dependencies(gps_localization_node ${catkin_EXPORTED_TARGETS})
target_include_directories(gps_localization_node PRIVATE ${CARMEN_INCLUDE_DIRS})
target_link_directories(gps_localization_node PRIVATE ${CARMEN_LIBRARY_DIRS})
target_link_libraries(gps_localization_node
  ${catkin_LIBRARIES}
  Boost::thread
  ${CARMEN_LIBRARIES}
)

# ─── fused_odometry_from_slam (injeta a pose do SC-LIO-SAM no CARMEN) ───────
# TESTE: publica carmen_fused_odometry_message a partir de /lio_sam/mapping/odometry.
# O localize_ackerman com -mapping_mode on converte isso em globalpos.
# Exige que o modulo `fused_odometry` do CARMEN esteja DESLIGADO.
add_executable(fused_odometry_from_slam_node src/fused_odometry_from_slam_node.cpp)
add_dependencies(fused_odometry_from_slam_node ${catkin_EXPORTED_TARGETS})
target_include_directories(fused_odometry_from_slam_node PRIVATE ${CARMEN_INCLUDE_DIRS})
target_link_directories(fused_odometry_from_slam_node PRIVATE ${CARMEN_LIBRARY_DIRS})
target_link_libraries(fused_odometry_from_slam_node
  ${catkin_LIBRARIES}
  Boost::thread
  ${CARMEN_LIBRARIES}
)

# ─── generate_poses_opt (grava poses_opt.dat a partir de TF + LiDAR) ────────
# Apague ou comente estas linhas:
# add_executable(generate_poses_opt src/generate_poses_opt.cpp)
# add_dependencies(generate_poses_opt ${catkin_EXPORTED_TARGETS})
# target_link_libraries(generate_poses_opt
#   ${catkin_LIBRARIES}
#   Boost::filesystem
# )

# ─── Install ──────────────────────────────────────────────────────────────
#install(TARGETS ipc_bridge_node pointcloud_node gps_localization_node generate_poses_opt
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(TARGETS ipc_bridge_node pointcloud_node gps_localization_node
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

set_target_properties(gps_localization_node PROPERTIES
  BUILD_RPATH ""
  INSTALL_RPATH ""
  SKIP_BUILD_RPATH FALSE
  BUILD_WITH_INSTALL_RPATH FALSE
)