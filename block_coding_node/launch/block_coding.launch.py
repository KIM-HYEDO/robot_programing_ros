import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('block_coding_node'),
            'param',
            'param.yaml'))
    lidar_use = LaunchConfiguration(
        'lidar_use',
        default=True,)
    lidar_mode = LaunchConfiguration(
        'lidar_use',
        default='raw')
    print(lidar_use)
    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path of parameter file'),
        DeclareLaunchArgument(
            'lidar_use',
            default_value=lidar_use,
            description='Whether to use lidar'),
        DeclareLaunchArgument(
            'lidar_mode',
            default_value=lidar_mode,
            description='lidar mode (raw,zero,trunc)'),
        Node(
            package='block_coding_node',
            executable='coding_node',
            name='coding_node',
            parameters=[param_dir],
            output='screen'),

        Node(
            package='motor_control',
            executable='motor_control',
            name='motor_control',
            parameters=[param_dir],
            output='screen'),
        Node(
            package='beagle_robot',
            executable='beagle_robot',
            name='beagle_robot',
            parameters=[param_dir,{'lidar_use': LaunchConfiguration('lidar_use')}],
            arguments=["--lidar-mode", LaunchConfiguration('lidar_mode')],
            output='screen'),
        # Node(
        #     package='beagle_camera',
        #     executable='camera_yolo',
        #     name='camera_yolo',
        #     parameters=[param_dir],
        #     output='screen'),
    ])
