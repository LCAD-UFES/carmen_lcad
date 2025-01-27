./carmenkillall.sh hercules/process-hercules-yolo-sensorbox3.ini
./carmenkillall.sh argos/process-argos-playback-webrtc.ini
./carmenkillall.sh process-volta_da_ufes-pid.ini
killall roslaunch rosmaster  rosout linear_actuator_controller can_to_ros_main vel_to_can_main open_manipulator_teleop_topic
killall pointcloud_to_laserscan_node robot_state_publisher rviz2 joy_node teleop_node twist_mux async_slam_toolbox_node can_to_wirelesscontroller_webrtc _ros2_daemon 
killall task_manager go2_driver_node
pkill -f ros2
pkill -f /opt/ros/foxy/lib
