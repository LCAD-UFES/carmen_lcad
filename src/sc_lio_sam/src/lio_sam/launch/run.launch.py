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
    for key in ('extrinsicRPY', 'extrinsicTrans'):
        if key in parameters:
            parameters[key] = [float(v) for v in parameters[key]]
    directory = Path(value('savePCDDirectory')).expanduser().resolve()
    if directory in (Path('/'), Path.home()):
        raise ValueError('savePCDDirectory must be a dedicated map directory')
    parameters.update(savePCDDirectory=str(directory)+'/', use_sim_time=boolean('use_sim_time'),
                      localizationMode=boolean('localizationMode'), useGpu=boolean('gpu'))
    for key in ('initialPoseX','initialPoseY','initialPoseZ','initialPoseRoll','initialPosePitch','initialPoseYaw'):
        parameters[key] = float(value(key))
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
    defaults={'use_sim_time':'false','localizationMode':'false','gpu':'false','rviz':'true',
              'rviz_software_rendering':'false',
              'initialPoseX':'0.0','initialPoseY':'0.0','initialPoseZ':'0.0',
              'initialPoseRoll':'0.0','initialPosePitch':'0.0','initialPoseYaw':'0.0'}
    return LaunchDescription([DeclareLaunchArgument('params_file'),DeclareLaunchArgument('savePCDDirectory')]+
        [DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=setup)])
