"""File-based LT-mapper processing with an explicit output directory."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
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
        config = yaml.safe_load(stream)
    parameters = {}
    def flatten(prefix, mapping):
        for key, item in mapping.items():
            name = prefix + key
            if isinstance(item, dict):
                flatten(name + '.', item)
            else:
                parameters[name] = item
    flatten('', config)
    parameters['map_reconstruct.output_dir'] = str(Path(value('output_dir')).expanduser().resolve()) + '/'
    parameters['use_sim_time'] = boolean('use_sim_time')
    parameters['keep_alive'] = boolean('keep_alive')
    return [Node(package='removert', executable='removert_reconstruct_feature_map', parameters=[parameters], output='screen')]


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('removert')) / 'config' / 'params_reconstruct.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_config),
        DeclareLaunchArgument('output_dir'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('keep_alive', default_value='false'),
        OpaqueFunction(function=setup)])
