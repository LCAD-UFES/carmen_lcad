import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    with open(LaunchConfiguration('params_file').perform(context)) as stream:
        parameters = yaml.safe_load(stream)['lio_sam']
    for key in ('sensor', 'surroundingKeyframeSize', 'localizationMode', 'useGpu',
                'savePCD', 'savePCDDirectory', 'map_directory'):
        parameters.pop(key, None)
    for key in ('extrinsicRPY', 'extrinsicTrans'):
        if key in parameters:
            parameters[key] = [float(value) for value in parameters[key]]
    parameters['use_sim_time'] = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    return [Node(package='lio_sam', executable='lio_sam_imageProjection',
                 output='screen', parameters=[parameters])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='true'), OpaqueFunction(function=setup)])
