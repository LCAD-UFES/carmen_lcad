"""Native CARMEN interfaces and optional recording/calibration tools."""
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def boolean(name):
        text = value(name).lower()
        if text not in ('true', 'false'):
            raise ValueError(name + ' must be true or false')
        return text == 'true'

    clock = {'use_sim_time': boolean('use_sim_time')}
    nodes = []

    def add(executable, parameters, arguments=None):
        node = Node(package='carmen_ros_bridge', executable=executable,
                    parameters=[parameters], arguments=arguments or [], output='screen')
        nodes.append(node)

    with open(value('params_file')) as stream:
        collision = yaml.safe_load(stream)['carmen_ros_bridge']
    gps_covariance_keys = ('gps_sigma_xy_quality_1', 'gps_sigma_xy_quality_2',
                           'gps_sigma_xy_quality_4', 'gps_sigma_xy_quality_5',
                           'gps_sigma_xy_default', 'gps_sigma_z_multiplier')
    # GPS uncertainty settings are passed only to the XYZ converter.
    gps_covariance = {key: float(collision.pop(key)) for key in gps_covariance_keys
                      if key in collision}
    for key in ('robot_collision_height', 'semi_trailer_collision_height', 'collision_safety_margin'):
        if key not in collision:
            raise ValueError('Missing carmen_ros_bridge.' + key)
    common = dict(clock, laser_frame_id=value('laser_frame_id'),
                  lidar_sensor_id=int(value('lidar_sensor_id')))
    add('carmen_to_ros_lidar_raw', dict(common,
        use_exact_lidar_timestamp=boolean('use_exact_lidar_timestamp')))
    add('ros_to_ros_lidar_collision_model', dict(common, **collision,
        base_frame_id=value('base_frame_id')))
    add('ros_to_ros_gps_msg_to_gps_odometry', dict(clock, gps_nrs=yaml.safe_load(value('gps_nrs')),
        lio_sam_gps_topic='/odometry/gps', gps_output_frame_id='utm',
        car_frame_id=value('base_frame_id'), gps_max_age=.3, heading_max_age=1.,
        v_max_age=1., use_latency_compensation=True))
    imu = dict(clock, base_frame_id=value('base_frame_id'), imu_frame_id=value('imu_frame_id'),
               publish_imu=boolean('publish_imu'), imu_rotation=yaml.safe_load(value('imu_rotation')))
    add('carmen_to_ros_imu_xsens', imu)
    add('carmen_to_ros_imu_lidar', dict(imu, warn_on_xsens=boolean('warn_on_xsens'),
        lidar_imu_sensor_id=int(value('lidar_sensor_id')), xsens_node_name=value('xsens_node_name')))
    gps = dict(clock, gps_frame_id=value('gps_frame_id'), publish_gps=boolean('publish_gps'))
    add('carmen_to_ros_gps_xyz', dict(gps, **gps_covariance))
    add('carmen_to_ros_gps_nmea', gps)
    add('carmen_to_ros_odometry', dict(clock, base_frame_id=value('base_frame_id'),
                                    publish_ackermann=boolean('publish_ackermann')))
    if boolean('fused_odometry'):
        parameters = dict(clock, world_origin_file=value('world_origin_file'),
                          base_frame_id=value('base_frame_id'), lidar_frame_id=value('laser_frame_id'))
        for key in ('origin_x', 'origin_y', 'origin_z', 'origin_yaw'):
            parameters[key] = float(value(key))
        add('ros_to_carmen_globalpos', parameters)
    if boolean('record_poses'):
        if not value('poses_output_file'):
            raise ValueError('record_poses requires poses_output_file')
        add('generate_poses_opt', clock, [value('poses_output_file')])
    # Calibration ends normally after its collection interval.
    actions = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='CARMEN bridge component exited'))])) for node in nodes]
    if boolean('calibrate_imu'):
        add('imu_noise_calibration_node.py', dict(clock, imu_topic=value('calibration_imu_topic'),
                                                 duration=float(value('calibration_duration'))))
    return actions + nodes


def generate_launch_description():
    defaults = {
        'use_sim_time': 'true', 'lidar_sensor_id': '0', 'laser_frame_id': 'velodyne',
        'base_frame_id': 'base_link', 'imu_frame_id': 'imu_link', 'gps_frame_id': 'utm',
        'use_exact_lidar_timestamp': 'false', 'gps_nrs': '[1, 2]',
        'publish_imu': 'true', 'publish_gps': 'true', 'publish_ackermann': 'true',
        'warn_on_xsens': 'true', 'xsens_node_name': 'carmen_to_ros_imu_xsens',
        'imu_rotation': '[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]',
        'fused_odometry': 'false', 'world_origin_file': '',
        'origin_x': '0.0', 'origin_y': '0.0', 'origin_z': '0.0', 'origin_yaw': '0.0',
        'record_poses': 'false', 'poses_output_file': '', 'calibrate_imu': 'false',
        'calibration_imu_topic': '/imu_raw', 'calibration_duration': '1800.0'}
    return LaunchDescription([DeclareLaunchArgument('params_file')] +
        [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()] +
        [OpaqueFunction(function=setup)])
