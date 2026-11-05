"""Standalone mapping validation; existing ROS1 processes remain unchanged."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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

    with open(value('params_file')) as stream:
        parameters = yaml.safe_load(stream)['lio_sam']
    for key in ('sensor', 'surroundingKeyframeSize', 'localizationMode', 'useGpu',
                'savePCD', 'savePCDDirectory', 'map_directory'):
        parameters.pop(key, None)
    for key in ('extrinsicRPY', 'extrinsicTrans'):
        if key in parameters:
            parameters[key] = [float(item) for item in parameters[key]]
    map_directory = value('map_directory')
    if not map_directory.strip():
        raise ValueError('map_directory must not be empty')
    output = Path(map_directory).expanduser().resolve()
    if output == Path('/') or output == Path.home():
        raise ValueError('map_directory must identify a dedicated map directory')
    parameters.update(map_directory=str(output) + '/',
                      localizationMode=boolean('localizationMode'),
                      use_sim_time=boolean('use_sim_time'), useGpu=boolean('gpu'))
    if value('posesEnable'):
        parameters['posesEnable'] = boolean('posesEnable')
    return [Node(package='lio_sam', executable='lio_sam_mapOptmization',
                 output='screen', parameters=[parameters])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('map_directory', description='Dedicated output/map directory; recreated in mapping mode'),
        DeclareLaunchArgument('localizationMode', choices=['true', 'false'],
                              description='Required: true to localize, false to build a map'),
        DeclareLaunchArgument('posesEnable', default_value='', choices=['', 'true', 'false'],
                              description='Override YAML pose recording; empty uses the YAML value'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gpu', default_value='false'),
        OpaqueFunction(function=setup)])
