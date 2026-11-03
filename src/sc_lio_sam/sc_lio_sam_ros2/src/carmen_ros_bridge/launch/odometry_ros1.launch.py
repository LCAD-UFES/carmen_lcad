from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    converter = Node(
        package='carmen_ros_bridge', executable='carmen_to_ros_odometry',
        output='screen', parameters=[{
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'publish_ackermann': ParameterValue(
                LaunchConfiguration('publish_ackermann'), value_type=bool)}])
    bridge = Node(package='carmen_ros_bridge', executable='odometry_ros2_to_ros1.py',
                  output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('base_frame_id', default_value='base_link'),
        DeclareLaunchArgument('publish_ackermann', default_value='true'),
        RegisterEventHandler(OnProcessExit(target_action=converter,
            on_exit=[EmitEvent(event=Shutdown(reason='Odometry converter exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='Odometry bridge exited'))])),
        bridge, converter])
