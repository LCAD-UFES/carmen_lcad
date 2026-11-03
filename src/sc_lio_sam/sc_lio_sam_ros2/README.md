# CARMEN ROS2 migration

This workspace uses ROS2 Lyrical. All carmen_ros_bridge nodes now run in ROS2. SC-LIO-SAM remains in ROS1 Noetic, connected through the topic bridges below.

## Build

From a shell without the ROS1 workspace sourced:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/carmen_lcad/src/sc_lio_sam/sc_lio_sam_ros2
CARMEN_HOME=~/carmen_lcad colcon build --base-paths src --packages-select carmen_ros_bridge
```

Use `--base-paths src`: the workspace root contains `CATKIN_IGNORE` so the
surrounding ROS1 workspace does not discover this ROS2 package.

## Run

Start ROS1 roscore, CARMEN central and param_daemon, then:

```bash
bash ~/carmen_lcad/src/sc_lio_sam/sc_lio_sam_ros2/run_odometry_ros1.sh
```

Arguments: `base_frame_id:=base_link`, `publish_ackermann:=true`.
The wrapper isolates ROS2 Python and library paths from the ROS1 environment.
`ROS_MASTER_URI`, `CENTRALHOST` and `ROS_DOMAIN_ID` are inherited.

Data flow:

```
CARMEN base_ackerman_odometry
  -> carmen_to_ros_odometry (ROS2 TwistStamped, /ackermann/odom_raw)
  -> odometry_ros2_to_ros1 (ROS2 subscriber)
  -> anonymous pipe -> ROS1 publisher (/ackermann/odom_raw)
  -> existing ROS1 consumers
```

The bridge is specific to TwistStamped, not a general replacement for
ros1_bridge. Its ROS1 helper runs in a separate process with Noetic sourced;
ROS1 and ROS2 Python libraries are never imported into the same process.
The local pipe carries seconds, nanoseconds, UTF-8 frame ID and six float64
components as JSON. ROS1 assigns its own header sequence counter (ROS2 has none).
The bridge does not modify timestamps or publish `/clock`.

`linear.x` remains speed in m/s; `angular.z` remains steering angle in radians,
not yaw rate. Other components remain zero for CARMEN odometry. ROS2 uses
reliable, volatile QoS with depth 200; ROS1 publisher queue depth is 200.
CARMEN retains `CARMEN_SUBSCRIBE_LATEST`. Start the ROS master before the bridge.
The launch shuts down both sides when either child exits; proccontrol can
restart the group. The ROS1 converter has been removed to avoid duplicates.

## Integration test

The test starts isolated ROS1 (11441), IPC (14841), ROS2 domain 141, and the
IARA parameter daemon. It uses synthetic odometry, not a vehicle log.
Do not use those ports/domain for other processes during the test.

```bash
c++ -I$CARMEN_HOME/include tests/publish_ipc_odometry.cpp \
  -L$CARMEN_HOME/lib -Wl,--start-group -lglobal -lipc -Wl,--end-group \
  -lz -o /tmp/publish_ipc_odometry
source /opt/ros/noetic/setup.bash
python3 tests/integration_odometry.py
```

It verifies exact speed, steering, timestamps, custom frame, all six Twist
components, bridge restart and `publish_ackermann=false`.

## IMU and GPS

Start the sensor converters and their ROS2 -> ROS1 bridge with:

```bash
bash ~/carmen_lcad/src/sc_lio_sam/sc_lio_sam_ros2/run_sensors_ros1.sh lidar_sensor_id:=5 use_sim_time:=true
```

Use the same `lidar_sensor_id` and `use_sim_time` as the LiDAR launch.
Other arguments are `publish_imu`, `publish_gps`, `warn_on_xsens`, `imu_rotation`,
`base_frame_id`, `imu_frame_id`, `gps_frame_id`, `xsens_node_name` and `gps_nrs`.
Both IMUs publish independently on `/imu_raw`. The LiDAR node processes the
warning subscription on its main thread, identifying Xsens by its DDS publisher
GID and configured node name. No sensor priority is introduced.

The sensor bridge forwards `/imu_raw`, `/gps/heading_raw`, GPS XYZ receiver
topics `/gps/xyz_raw/gps_<nr>`, and `/tf_static`. It discovers additional GPS
receivers dynamically; receivers listed in `gps_nrs` (default `[1, 2]`) are
subscribed at startup. A newly discovered volatile topic can lose its earliest
samples during discovery, like any ROS publisher/subscriber startup.
Static transforms are received with transient-local durability, accumulated by
child frame and latched on ROS1 for late subscribers.

`gps_localization_node` now runs in ROS2 and consumes native GPS and odometry topics.
The original ROS1 package has been removed. IARA and ARGOS processes start
all three converter/bridge groups, preserving their startup/watch settings.

### Differential sensor test

Before removal, the four ROS1 reference executables were archived in
`/tmp/ros1_sensor_reference` on this machine. To repeat the differential test on
another machine, build the pre-migration ROS1 executables in a separate checkout
and set `ROS1_SENSOR_REFERENCE` to their directory.

```bash
c++ -I$CARMEN_HOME/include tests/publish_ipc_sensors.cpp \
  -L$CARMEN_HOME/lib -Wl,--start-group -limu_interface -lglobal -lipc \
  -Wl,--end-group -lz -o /tmp/publish_ipc_sensors
source /opt/ros/noetic/setup.bash
ROS1_SENSOR_REFERENCE=/tmp/ros1_sensor_reference python3 tests/integration_sensors.py
```

The test uses ROS1 port 11442, IPC port 14842 and ROS2 domain 142. It compares
serialized ROS1 messages byte for byte (excluding the ROS1 sequence counter),
including nonidentity IMU rotation, covariances, GPS quality and validity,
receivers 1/2/7, static TF for late subscribers, simultaneous IMUs and publication
disable flags. These are synthetic tests, not a full vehicle-log/map replay.

## LiDAR, GPS fusion and SLAM feedback

After starting roscore, CARMEN central and param_daemon, start all three groups:

```bash
bash run_odometry_ros1.sh
bash run_sensors_ros1.sh lidar_sensor_id:=0 use_sim_time:=true
bash run_remaining_ros1.sh params_file:=$CARMEN_HOME/data/iara/parameters/param.yaml lidar_sensor_id:=0 use_sim_time:=true
```

The last group runs `carmen_to_ros_lidar_raw`,
`ros_to_ros_lidar_collision_model`, `gps_localization_node` and the bidirectional
bridge. Collision parameters are read once from the vehicle YAML's `ipc_bridge`
section; required heights must be present. The launch also sets ROS1
`/use_sim_time`, matching the previous launch. Sensor ID, frames, GPS receivers
and `use_exact_lidar_timestamp` remain launch arguments.

ROS2 → ROS1: `/points_raw`, `/points_raw_no_vehicle`, `/clock`,
`/carmen/scan_time_reference`, `/odometry/gps` and vehicle/collision markers.
ROS1 → ROS2: `/odometry/imu`, `/lio_sam/mapping/odometry`, `/tf`.
`/initialpose` travels both ways, excluding messages from the bridge itself.
The sensor bridge forwards `/tf_static`, including the LiDAR and GPS transforms.
The bridges cover these explicit topics; custom topic names require updating
`remaining_protocol.py` or matching remappings on both endpoints.

`remaining_ros2_bridge.py` and `remaining_ros1_endpoint.py` exchange typed JSON
over anonymous pipes, with base64 for PointCloud2 bytes. ROS1 and ROS2 run in
separate environments. Mesh markers retain ROS1 latching. This compatibility
layer adds serialization work; throughput on full vehicle logs still needs
validation.

Standalone migrated utilities:

```bash
bash run_ros2_node.sh fused_odometry_from_slam_node --ros-args -p world_origin_file:=/path/to/world_origin.txt -p use_sim_time:=true
bash run_ros2_node.sh generate_poses_opt /tmp/poses.dat --ros-args -p use_sim_time:=true
bash run_ros2_node.sh imu_noise_calibration_node.py --ros-args -p duration:=1800.0
```

### Remaining-node differential test

`tests/integration_remaining.py` uses isolated ROS1 port 11443, IPC port 14843
and ROS2 domain 143. It compares 20 synthetic 128-ray scans and GPS output
against the archived ROS1 executables, excluding only header sequence numbers.
It checks collision removal, timestamps, reverse odometry/TF, initialpose loop
prevention, fused CARMEN odometry, pose recording and clean shutdown.
Reference binaries are currently in `/tmp/ros1_remaining_reference`; rebuild
the pre-migration package separately and set `ROS1_REMAINING_REFERENCE` if needed.

```bash
c++ -I$CARMEN_HOME/include tests/publish_ipc_lidar.cpp -L$CARMEN_HOME/lib -Wl,--start-group -lvelodyne_interface -lparam_interface -lglobal -lipc -Wl,--end-group -lz -o /tmp/publish_ipc_lidar
c++ -I$CARMEN_HOME/include tests/receive_fused.cpp -L$CARMEN_HOME/lib -Wl,--start-group -lfused_odometry_interface -lglobal -lipc -Wl,--end-group -lz -o /tmp/receive_fused
source /opt/ros/noetic/setup.bash
python3 tests/integration_remaining.py
```

These checks use synthetic input. Full proccontrol playback and RViz map
validation have not been performed for this migration.
