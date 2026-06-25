cmake_minimum_required(VERSION 3.5)
project(carmen_ipc_bridge)

# ─── C++ Standard ────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# ─── ROS2 core ───────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ─── Optional: tf2_geometry_msgs ─────────────────────────────────────────────
find_package(tf2_geometry_msgs QUIET)
if(tf2_geometry_msgs_FOUND)
  message(STATUS "tf2_geometry_msgs found — conversão quaternion habilitada")
  add_compile_definitions(HAVE_TF2_GEOMETRY_MSGS)
else()
  message(WARNING
    "tf2_geometry_msgs NOT found.\n"
    "Instale com: sudo apt install ros-foxy-tf2-geometry-msgs")
endif()

# ─── CARMEN / IPC ────────────────────────────────────────────────────────────
if(NOT DEFINED ENV{CARMEN_HOME})
  message(FATAL_ERROR
    "CARMEN_HOME não está definida.\n"
    "Execute: export CARMEN_HOME=~/carmen_lcad   e recompile.")
endif()

set(CARMEN_HOME $ENV{CARMEN_HOME})
message(STATUS "CARMEN_HOME = ${CARMEN_HOME}")

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

if(NOT EXISTS "${CARMEN_HOME}/include/carmen/carmen.h")
  message(FATAL_ERROR
    "carmen.h não encontrado em ${CARMEN_HOME}/include/carmen/carmen.h\n"
    "Verifique que o CARMEN está compilado e CARMEN_HOME está correto.")
endif()

link_directories(${CARMEN_LIBRARY_DIRS})

# Grupo --start-group / --end-group para resolver dependências circulares
# entre .a estáticos do CARMEN
set(CARMEN_LIBRARIES
  -Wl,--start-group
  ipc global param_interface
  localize_ackerman_interface
  velodyne_interface
  fused_odometry_interface
  map_interface map_util map_io map_server_interface
  mapper_interface mapper
  prob_models grid_mapping
  gps_xyz_interface
  moving_objects_interface
  rotation_geometry
  xsens_interface
  xsens_core
  tf BulletCollision BulletDynamics LinearMath
  -Wl,--end-group
)

# ─── Helper macro ────────────────────────────────────────────────────────────
macro(configure_bridge_target TARGET)
  target_include_directories(${TARGET} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CARMEN_INCLUDE_DIRS}
  )
  target_link_libraries(${TARGET} ${CARMEN_LIBRARIES})
  install(TARGETS ${TARGET} DESTINATION lib/${PROJECT_NAME})
endmacro()

# ─── ipc_bridge_node ─────────────────────────────────────────────────────────
add_executable(ipc_bridge_node
  src/ipc_bridge_node.cpp
)

set(IPC_BRIDGE_DEPS
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)
if(tf2_geometry_msgs_FOUND)
  list(APPEND IPC_BRIDGE_DEPS tf2_geometry_msgs)
endif()

ament_target_dependencies(ipc_bridge_node ${IPC_BRIDGE_DEPS})
configure_bridge_target(ipc_bridge_node)

# ─── Install recursos ─────────────────────────────────────────────────────────
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()