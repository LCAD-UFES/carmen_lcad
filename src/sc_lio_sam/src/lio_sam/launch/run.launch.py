"""Native SC-LIO-SAM pipeline with the existing vehicle launch arguments."""
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
        parameters = yaml.safe_load(stream)['lio_sam']
    for key in ('sensor', 'surroundingKeyframeSize', 'localizationMode', 'useGpu',
                'savePCD', 'savePCDDirectory', 'map_directory'):
        parameters.pop(key, None)
    for key in ('extrinsicRPY', 'extrinsicTrans'):
        if key in parameters:
            parameters[key] = [float(v) for v in parameters[key]]
    map_directory = value('map_directory')
    if not map_directory.strip():
        raise ValueError('map_directory must not be empty')
    directory = Path(map_directory).expanduser().resolve()
    if directory in (Path('/'), Path.home()):
        raise ValueError('map_directory must be a dedicated map directory')
    parameters.update(map_directory=str(directory)+'/', use_sim_time=boolean('use_sim_time'),
                      localizationMode=boolean('localizationMode'), useGpu=boolean('gpu'))
    if value('posesEnable'):
        parameters['posesEnable'] = boolean('posesEnable')
    nodes = [Node(package='lio_sam', executable='lio_sam_'+kind, parameters=[parameters],
                  output='screen', respawn=True, respawn_delay=1.)
             for kind in ('imuPreintegration','imageProjection','featureExtraction','mapOptmization')]
    share = Path(get_package_share_directory('lio_sam'))
    nodes.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description':(share/'config/robot.urdf').read_text(),
                     'use_sim_time':boolean('use_sim_time')}], output='screen',respawn=True))
    if boolean('rviz'):
        rviz_environment = {'QT_QPA_PLATFORM': 'xcb'}
        if boolean('rviz_software_rendering'):
            rviz_environment.update(LIBGL_ALWAYS_SOFTWARE='1', __GLX_VENDOR_LIBRARY_NAME='mesa',
                                    QT_QPA_PLATFORMTHEME='generic', QT_STYLE_OVERRIDE='Fusion')
        nodes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'config/rviz.rviz')],
            parameters=[{'use_sim_time':boolean('use_sim_time')}],output='screen',
            additional_env=rviz_environment))
    return nodes


def generate_launch_description():
    defaults={'use_sim_time':'true','gpu':'false','rviz':'true',
              'rviz_software_rendering':'false'}
    return LaunchDescription([DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('map_directory', description='Required map directory; recreated in mapping mode'),
        DeclareLaunchArgument('localizationMode', choices=['true', 'false'],
                              description='Required: true to localize, false to build a map'),
        DeclareLaunchArgument('posesEnable', default_value='', choices=['', 'true', 'false'],
                              description='Override YAML pose recording; empty uses the YAML value')]+
        [DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=setup)])
