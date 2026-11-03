from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'base_frame_id': 'base_link', 'imu_frame_id': 'imu_link', 'gps_frame_id': 'utm',
        'publish_imu': 'true', 'publish_gps': 'true', 'warn_on_xsens': 'true',
        'lidar_sensor_id': '0', 'use_sim_time': 'false',
        'xsens_node_name': 'carmen_to_ros_imu_xsens', 'gps_nrs': '[1, 2]',
        'imu_rotation': '[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]'}
    imu = {key: LaunchConfiguration(key) for key in ('base_frame_id', 'imu_frame_id')}
    for key in ('publish_imu', 'use_sim_time'):
        imu[key] = ParameterValue(LaunchConfiguration(key), value_type=bool)
    imu['imu_rotation'] = LaunchConfiguration('imu_rotation')
    gps = {'gps_frame_id': LaunchConfiguration('gps_frame_id'),
           'publish_gps': ParameterValue(LaunchConfiguration('publish_gps'), value_type=bool)}
    lidar = dict(imu)
    lidar.update(warn_on_xsens=ParameterValue(LaunchConfiguration('warn_on_xsens'), value_type=bool),
                 lidar_imu_sensor_id=ParameterValue(LaunchConfiguration('lidar_sensor_id'), value_type=int),
                 xsens_node_name=LaunchConfiguration('xsens_node_name'))
    nodes = [Node(package='carmen_ros_bridge', executable='sensors_ros2_to_ros1.py',
                  parameters=[{'gps_nrs': LaunchConfiguration('gps_nrs')}], output='screen')]
    for kind, parameters in [('imu_xsens', imu), ('imu_lidar', lidar), ('gps_xyz', gps), ('gps_nmea', gps)]:
        nodes.append(Node(package='carmen_ros_bridge', executable='carmen_to_ros_'+kind,
                          parameters=[parameters], output='screen'))
    actions = [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]
    for node in nodes:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='Sensor bridge component exited'))])))
    return LaunchDescription(actions + nodes)
