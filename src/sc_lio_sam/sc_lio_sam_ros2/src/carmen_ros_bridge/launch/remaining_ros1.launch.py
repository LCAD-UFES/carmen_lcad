import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def value(name): return LaunchConfiguration(name).perform(context)
    def boolean(name):
        text = value(name).lower()
        if text not in ('true', 'false'): raise ValueError(name+' must be true or false')
        return text == 'true'
    with open(value('params_file')) as stream:
        config = yaml.safe_load(stream)
    collision = config['ipc_bridge']
    for key in ('robot_collision_height', 'semi_trailer_collision_height', 'collision_safety_margin'):
        if key not in collision: raise ValueError('Missing ipc_bridge.'+key)
    common = {'use_sim_time': boolean('use_sim_time'), 'laser_frame_id': value('laser_frame_id'),
              'lidar_sensor_id': int(value('lidar_sensor_id'))}
    filter_params = dict(common, **collision)
    filter_params['base_frame_id'] = value('base_frame_id')
    nodes = [
        Node(package='carmen_ros_bridge', executable='remaining_ros2_bridge.py', output='screen',
             parameters=[{'ros1_use_sim_time': boolean('use_sim_time')}]),
        Node(package='carmen_ros_bridge', executable='carmen_to_ros_lidar_raw', output='screen',
             parameters=[dict(common, use_exact_lidar_timestamp=boolean('use_exact_lidar_timestamp'))]),
        Node(package='carmen_ros_bridge', executable='ros_to_ros_lidar_collision_model', output='screen',
             parameters=[filter_params]),
        Node(package='carmen_ros_bridge', executable='gps_localization_node', output='screen', parameters=[{
            'use_sim_time': boolean('use_sim_time'), 'gps_nrs': yaml.safe_load(value('gps_nrs')),
            'lio_sam_gps_topic': '/odometry/gps', 'gps_output_frame_id': 'utm',
            'car_frame_id': value('base_frame_id'), 'gps_max_age': .3, 'heading_max_age': 1.,
            'v_max_age': 1., 'use_latency_compensation': True}])]
    actions = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='Remaining bridge component exited'))])) for node in nodes]
    return actions + nodes


def generate_launch_description():
    defaults = {'use_sim_time': 'false', 'lidar_sensor_id': '0', 'laser_frame_id': 'velodyne',
                'base_frame_id': 'base_link', 'use_exact_lidar_timestamp': 'false', 'gps_nrs': '[1, 2]'}
    return LaunchDescription([DeclareLaunchArgument('params_file')] +
        [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()] +
        [OpaqueFunction(function=setup)])
