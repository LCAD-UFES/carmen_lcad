cmake_minimum_required(VERSION 3.20)
project(carmen_ros_bridge)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
set(CARMEN_HOME "$ENV{CARMEN_HOME}" CACHE PATH "CARMEN installation")
if(NOT EXISTS "${CARMEN_HOME}/include/carmen/carmen.h")
  message(FATAL_ERROR "Set CARMEN_HOME to the CARMEN installation")
endif()
add_executable(carmen_to_ros_odometry src/carmen_to_ros_odometry.cpp)
target_include_directories(carmen_to_ros_odometry PRIVATE "${CARMEN_HOME}/include")
target_link_directories(carmen_to_ros_odometry PRIVATE "${CARMEN_HOME}/lib")
target_link_libraries(carmen_to_ros_odometry PRIVATE rclcpp::rclcpp ${geometry_msgs_TARGETS}
  -Wl,--start-group base_ackerman_interface param_interface global ipc -Wl,--end-group z m)
target_compile_options(carmen_to_ros_odometry PRIVATE -Wall -Wextra -Wno-unused-parameter)
install(TARGETS carmen_to_ros_odometry DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/odometry_ros2_to_ros1.py scripts/odometry_ros1_sink.py
  scripts/run_ros1_sink.sh scripts/sensors_ros2_to_ros1.py scripts/sensors_ros1_sink.py
  scripts/run_sensors_ros1_sink.sh DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
foreach(sensor imu_xsens imu_lidar gps_xyz gps_nmea)
  set(target carmen_to_ros_${sensor})
  add_executable(${target} src/${target}.cpp)
  target_compile_definitions(${target} PRIVATE MAKE_IN_CARMEn)
  target_include_directories(${target} PRIVATE include "${CARMEN_HOME}/include")
  target_link_directories(${target} PRIVATE "${CARMEN_HOME}/lib")
  target_link_libraries(${target} PRIVATE rclcpp::rclcpp ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} tf2::tf2 tf2_ros::tf2_ros
    -Wl,--start-group xsens_interface imu_interface gps_xyz_interface gps_nmea_interface
    param_interface global ipc -Wl,--end-group z m)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wno-unused-parameter)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()
find_package(visualization_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost CONFIG REQUIRED COMPONENTS filesystem)
foreach(target carmen_to_ros_lidar_raw ros_to_ros_lidar_collision_model gps_localization_node fused_odometry_from_slam_node generate_poses_opt)
  add_executable(${target} src/${target}.cpp)
  target_compile_definitions(${target} PRIVATE MAKE_IN_CARMEn)
  target_include_directories(${target} PRIVATE include "${CARMEN_HOME}/include"
    "${CARMEN_HOME}/sharedlib/libtf/src" "${CARMEN_HOME}/sharedlib/prob_models" /usr/include/bullet)
  target_link_directories(${target} PRIVATE "${CARMEN_HOME}/lib"
    "${CARMEN_HOME}/sharedlib/libtf/src" "${CARMEN_HOME}/sharedlib/prob_models")
  target_link_libraries(${target} PRIVATE rclcpp::rclcpp ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${visualization_msgs_TARGETS} ${rosgraph_msgs_TARGETS}
    tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs Boost::filesystem
    -Wl,--start-group ipc global param_interface velodyne_interface base_ackerman_interface
    gps_xyz_interface gps_nmea_interface fused_odometry_interface collision_detection localize_ackerman_interface
    map_interface map_util map_io prob_models grid_mapping tf BulletCollision BulletDynamics BulletSoftBody LinearMath rt z
    -Wl,--end-group)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()
install(PROGRAMS scripts/remaining_ros2_bridge.py scripts/remaining_ros1_endpoint.py
  scripts/run_remaining_ros1.sh DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/remaining_protocol.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/imu_noise_calibration_node.py DESTINATION lib/${PROJECT_NAME})
ament_package()
