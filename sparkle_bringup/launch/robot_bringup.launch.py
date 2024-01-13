from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from glob import glob
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='localization_data',
            executable='odometry_publisher',
            name='odometry_publisher'
        ),
        Node(
            package='motor_controller',
            executable='motor_driver',
            name='motor_driver'
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0']
        ),
        Node(
            package='bno055',
            executable='bno055',
            arguments=['--ros-args', '--params-file', './src/bno055/bno055/params/bno055_params_i2c.yaml']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0.2176', '0', '0', '0', 'base_link', 'imu_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0.3224', '0', '0', '0', 'base_link', 'laser']
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['0.08166', '0', '0.2316', '0', '0', '0', 'base_link', 'oak']
        # ),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type':'serial',
                         'serial_port': '/dev/ttyUSB1',
                         'serial_baudrate': 115200,
                         'frame_id': 'laser',
                         'inverted': False,
                         'angle_compensate': True}],
            output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('depthai_ros_driver'), 'launch', 'pointcloud.launch.py')),
            launch_arguments={'name': 'oak',
                            'parent_frame': 'base_link',
                            'cam_pos_x': '0.08166',
                            'cam_pos_y': '0',
                            'cam_pos_z': '0.2316',
                            'cam_roll': '0',
                            'cam_pitch': '0',
                            'cam_yaw': '0',
                            'params_file': os.path.join(get_package_share_directory('sparkle_bringup'), 'config', 'camera.yaml')}.items()),
    ])