import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from sensor_nodes import sensor_nodes


def generate_launch_description():
    nav2 = get_package_share_directory('nav2_bringup')
    nodes = sensor_nodes()
    nodes.append(Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                      name='slam_toolbox', output='screen',
                      parameters=['/run/ugv-nav/slam.yaml']))
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='lifecycle_manager_slam', output='screen',
                      parameters=[{'autostart': True, 'node_names': ['slam_toolbox']}]))
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'false', 'autostart': 'true',
                          'params_file': '/run/ugv-nav/nav2_params.yaml',
                          'use_composition': 'False'}.items()))
    return LaunchDescription(nodes)
