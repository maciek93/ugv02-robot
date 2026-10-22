from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def sensor_nodes():
    return [
        Node(package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node',
             name='ld06', output='screen', parameters=[{
                 'product_name': 'LDLiDAR_LD06', 'topic_name': 'scan',
                 'frame_id': 'base_laser',
                 'port_name': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                 'port_baudrate': 230400, 'laser_scan_dir': True,
                 'enable_angle_crop_func': False}]),
        ExecuteProcess(cmd=['python3', '/opt/robot/ros2/tof_node.py'], output='screen'),
        ExecuteProcess(cmd=['python3', '/opt/robot/ros2/robot_bridge.py'], output='screen'),
    ]
