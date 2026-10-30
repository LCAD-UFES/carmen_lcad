cmake_minimum_required(VERSION 3.0.2)
project(carmen_ros_bridge)

option(MAKE_IN_CARMEn "Build against the Carmen APIs instead of Astro" OFF)
if(MAKE_IN_CARMEn)
  add_compile_definitions(MAKE_IN_CARMEn)
endif()

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
if(MAKE_IN_CARMEn)
  set(IPC_HOME_ENV CARMEN_HOME)
else()
  set(IPC_HOME_ENV ASTRO_HOME)
endif()
if(NOT DEFINED ENV{${IPC_HOME_ENV}})
  message(FATAL_ERROR "${IPC_HOME_ENV} não está definida.")
endif()

set(CARMEN_HOME $ENV{${IPC_HOME_ENV}})

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
set(IPC_SHARED_MEMORY_LIBRARIES)
if(NOT MAKE_IN_CARMEn)
  set(IPC_SHARED_MEMORY_LIBRARIES shm_map shm_manager)
endif()

set(CARMEN_LIBRARIES
  -Wl,--start-group
  ipc global param_interface
  velodyne_interface xsens_interface imu_interface base_ackerman_interface
  gps_xyz_interface gps_nmea_interface fused_odometry_interface
  collision_detection ${IPC_SHARED_MEMORY_LIBRARIES} localize_ackerman_interface
  map_interface map_util map_io
  prob_models grid_mapping
  tf BulletCollision BulletDynamics BulletSoftBody LinearMath rt z
  -Wl,--end-group
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

# Carmen to ROS Xsens IMU.
add_executable(carmen_to_ros_imu_xsens src/carmen_to_ros_imu_xsens.cpp)
add_dependencies(carmen_to_ros_imu_xsens ${catkin_EXPORTED_TARGETS})
target_include_directories(carmen_to_ros_imu_xsens PRIVATE ${CARMEN_INCLUDE_DIRS})
target_link_directories(carmen_to_ros_imu_xsens PRIVATE ${CARMEN_LIBRARY_DIRS})
target_link_libraries(carmen_to_ros_imu_xsens ${catkin_LIBRARIES} ${CARMEN_LIBRARIES})

# Carmen to ROS LiDAR IMU.
add_executable(carmen_to_ros_imu_lidar src/carmen_to_ros_imu_lidar.cpp)
add_dependencies(carmen_to_ros_imu_lidar ${catkin_EXPORTED_TARGETS})
target_include_directories(carmen_to_ros_imu_lidar PRIVATE ${CARMEN_INCLUDE_DIRS})
target_link_directories(carmen_to_ros_imu_lidar PRIVATE ${CARMEN_LIBRARY_DIRS})
target_link_libraries(carmen_to_ros_imu_lidar ${catkin_LIBRARIES} ${CARMEN_LIBRARIES})

# Carmen to ROS Ackermann odometry.
add_executable(carmen_to_ros_odometry src/carmen_to_ros_odometry.cpp)
add_dependencies(carmen_to_ros_odometry ${catkin_EXPORTED_TARGETS})
target_include_directories(carmen_to_ros_odometry PRIVATE ${CARMEN_INCLUDE_DIRS})
target_link_directories(carmen_to_ros_odometry PRIVATE ${CARMEN_LIBRARY_DIRS})
target_link_libraries(carmen_to_ros_odometry ${catkin_LIBRARIES} ${CARMEN_LIBRARIES})

# Carmen to ROS GPS XYZ.
add_executable(carmen_to_ros_gps_xyz src/carmen_to_ros_gps_xyz.cpp)
add_dependencies(carmen_to_ros_gps_xyz ${catkin_EXPORTED_TARGETS})
target_include_directories(carmen_to_ros_gps_xyz PRIVATE ${CARMEN_INCLUDE_DIRS})
target_link_directories(carmen_to_ros_gps_xyz PRIVATE ${CARMEN_LIBRARY_DIRS})
target_link_libraries(carmen_to_ros_gps_xyz ${catkin_LIBRARIES} ${CARMEN_LIBRARIES})

# Carmen to ROS GPS NMEA HDT.
add_executable(carmen_to_ros_gps_nmea src/carmen_to_ros_gps_nmea.cpp)
add_dependencies(carmen_to_ros_gps_nmea ${catkin_EXPORTED_TARGETS})
target_include_directories(carmen_to_ros_gps_nmea PRIVATE ${CARMEN_INCLUDE_DIRS})
target_link_directories(carmen_to_ros_gps_nmea PRIVATE ${CARMEN_LIBRARY_DIRS})
target_link_libraries(carmen_to_ros_gps_nmea ${catkin_LIBRARIES} ${CARMEN_LIBRARIES})

# Raw LiDAR conversion and ROS collision filtering.
add_executable(carmen_to_ros_lidar_raw src/carmen_to_ros_lidar_raw.cpp)
add_dependencies(carmen_to_ros_lidar_raw ${catkin_EXPORTED_TARGETS})
target_include_directories(carmen_to_ros_lidar_raw PRIVATE ${CARMEN_INCLUDE_DIRS})
target_link_directories(carmen_to_ros_lidar_raw PRIVATE ${CARMEN_LIBRARY_DIRS})
target_link_libraries(carmen_to_ros_lidar_raw ${catkin_LIBRARIES} ${CARMEN_LIBRARIES})

add_executable(ros_to_ros_lidar_collision_model src/ros_to_ros_lidar_collision_model.cpp)
add_dependencies(ros_to_ros_lidar_collision_model ${catkin_EXPORTED_TARGETS})
target_include_directories(ros_to_ros_lidar_collision_model PRIVATE ${CARMEN_INCLUDE_DIRS})
target_link_directories(ros_to_ros_lidar_collision_model PRIVATE ${CARMEN_LIBRARY_DIRS})
target_link_libraries(ros_to_ros_lidar_collision_model ${catkin_LIBRARIES} ${CARMEN_LIBRARIES})

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
install(TARGETS carmen_to_ros_gps_nmea carmen_to_ros_gps_xyz carmen_to_ros_odometry carmen_to_ros_imu_lidar carmen_to_ros_imu_xsens carmen_to_ros_lidar_raw ros_to_ros_lidar_collision_model gps_localization_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

set_target_properties(gps_localization_node PROPERTIES
  BUILD_RPATH ""
  INSTALL_RPATH ""
  SKIP_BUILD_RPATH FALSE
  BUILD_WITH_INSTALL_RPATH FALSE
)
