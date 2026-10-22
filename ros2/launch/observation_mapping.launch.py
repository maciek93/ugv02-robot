"""Build a provisional SLAM map from manually driven, observed odometry.

Nav2 and metric movement authority are intentionally absent in this mode.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

from sensor_nodes import sensor_nodes


def generate_launch_description():
    return LaunchDescription([
        *sensor_nodes(),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen',
             parameters=['/run/ugv-nav/slam.yaml']),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_slam', output='screen',
             parameters=[{'autostart': True, 'node_names': ['slam_toolbox']}]),
    ])
