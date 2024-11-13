./carmenkillall.sh hercules/process-hercules-yolo-sensorbox3.ini
./carmenkillall.sh process-volta_da_ufes-pid.ini
killall roslaunch rosmaster  rosout linear_actuator_controller can_to_ros_main vel_to_can_main open_manipulator_teleop_topic
killall task_manager
