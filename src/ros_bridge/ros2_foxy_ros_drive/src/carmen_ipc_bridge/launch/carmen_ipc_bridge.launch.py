"""
carmen_ipc_bridge.launch.py

Launch do nó bridge standalone CARMEN IPC → ROS2.
Sem slam_toolbox. Compatível com LIO-SAM.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    ipc_host_arg = DeclareLaunchArgument(
        'ipc_host',
        default_value='localhost',
        description='Host do central IPC do CARMEN'
    )

    bridge_node = Node(
        package='carmen_ipc_bridge',
        executable='ipc_bridge_node',
        name='carmen_ipc_bridge',
        output='screen',
        parameters=[{
            # ── Frames TF ────────────────────────────────────────
            'base_frame_id':  'base_link',
            'odom_frame_id':  'odom',
            'laser_frame_id': 'velodyne',
            'imu_frame_id':   'imu_link',

            # ── LiDAR ────────────────────────────────────────────
            'max_laser_range':    120.0,
            'laser_min_angle':    -3.14159,
            'laser_max_angle':     3.14159,
            'laser_num_beams':     360,

            # ── Tópicos a publicar ───────────────────────────────
            # True: publica /velodyne_points (PointCloud2) para LIO-SAM
            'publish_pointcloud': True,
            # True: publica /scan (LaserScan) para visualização / 2D SLAM
            'publish_laserscan':  True,
            # True: publica /imu/data — ative se o CARMEN tiver IMU
            'publish_imu':        False,

            # ── Odometria ────────────────────────────────────────
            # Distância entre eixos [m] — Argos ~2.625m
            'wheelbase': 2.625,

            # ── IPC ──────────────────────────────────────────────
            'ipc_host': LaunchConfiguration('ipc_host'),
        }]
    )

    return LaunchDescription([
        ipc_host_arg,
        bridge_node,
    ])