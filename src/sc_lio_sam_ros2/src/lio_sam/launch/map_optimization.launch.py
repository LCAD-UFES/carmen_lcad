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
    for key in ('extrinsicRPY', 'extrinsicTrans'):
        if key in parameters:
            parameters[key] = [float(item) for item in parameters[key]]
    output = Path(value('savePCDDirectory')).expanduser().resolve()
    if output == Path('/') or output == Path.home():
        raise ValueError('savePCDDirectory must identify a dedicated map directory')
    parameters.update(savePCDDirectory=str(output) + '/',
                      localizationMode=boolean('localizationMode'),
                      use_sim_time=boolean('use_sim_time'), useGpu=boolean('gpu'))
    return [Node(package='lio_sam', executable='lio_sam_mapOptmization',
                 output='screen', parameters=[parameters])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('savePCDDirectory', description='Dedicated output/map directory; recreated in mapping mode'),
        DeclareLaunchArgument('localizationMode', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('gpu', default_value='false'),
        OpaqueFunction(function=setup)])
