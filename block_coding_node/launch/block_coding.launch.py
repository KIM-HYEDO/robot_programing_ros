import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def str_to_bool(value):
    return value.lower() in ['true', '1', 'yes']

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
        'lidar_mode',
        default='raw')
    camera_use = LaunchConfiguration(
        'camera_use',
        default=False)
    launch_des = []
    launch_des.append(DeclareLaunchArgument(
        'param_dir',
        default_value=param_dir,
        description='Full path of parameter file'))
    launch_des.append(DeclareLaunchArgument(
        'lidar_use',
        default_value=lidar_use,
        description='Whether to use lidar'))
    launch_des.append(DeclareLaunchArgument(
        'lidar_mode',
        default_value=lidar_mode,
        description='lidar mode (raw,zero,trunc)'))
    launch_des.append(DeclareLaunchArgument(
        'camera_use',
        default_value=camera_use,
        description='Whether to use camera'))
    launch_des.append(Node(
        package='block_coding_node',
        executable='coding_node',
        name='coding_node',
        parameters=[param_dir],
        output='screen'))
    launch_des.append(Node(
        package='motor_control',
        executable='motor_control',
        name='motor_control',
        parameters=[param_dir],
        output='screen'))
    launch_des.append(Node(
        package='beagle_robot',
        executable='beagle_robot',
        name='beagle_robot',
        parameters=[param_dir, {
            'lidar_use': LaunchConfiguration('lidar_use')}],
        arguments=["--lidar-mode", LaunchConfiguration('lidar_mode')],
        output='screen'))

    print(type(camera_use))
    if camera_use:
        launch_des.append(Node(
            package='beagle_camera',
            executable='camera_yolo',
            name='camera_yolo',
            parameters=[param_dir],
            output='screen'))
    return LaunchDescription(launch_des)
