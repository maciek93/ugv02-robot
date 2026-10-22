from launch import LaunchDescription
from sensor_nodes import sensor_nodes


def generate_launch_description():
    return LaunchDescription(sensor_nodes())
