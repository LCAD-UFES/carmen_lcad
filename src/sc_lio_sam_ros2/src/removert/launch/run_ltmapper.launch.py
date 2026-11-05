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
    parameters['removert.save_pcd_directory'] = str(Path(value('save_pcd_directory')).expanduser().resolve()) + '/'
    parameters['use_sim_time'] = boolean('use_sim_time')
    parameters['keep_alive'] = boolean('keep_alive')
    nodes = [Node(package='removert', executable='removert_removert', parameters=[parameters], output='screen')]
    if boolean('rviz'):
        share = Path(get_package_share_directory('removert'))
        nodes.append(Node(package='rviz2', executable='rviz2',
            arguments=['-d', str(share / 'config' / 'removert_visualization.rviz')],
            parameters=[{'use_sim_time': parameters['use_sim_time']}],
            additional_env={'QT_QPA_PLATFORM': 'xcb'}, output='screen'))
    return nodes


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('removert')) / 'config' / 'params_ltmapper.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_config),
        DeclareLaunchArgument('save_pcd_directory'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('keep_alive', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=setup)])
