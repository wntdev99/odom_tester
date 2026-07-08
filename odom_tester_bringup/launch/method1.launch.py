"""방법 ① (odom_compare)를 /method1 네임스페이스로 실행."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('odom_tester_bringup'),
        'config', 'method1.yaml')

    config = LaunchConfiguration('config')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_cfg,
                              description='method1 파라미터 YAML 경로'),
        Node(
            package='odom_compare',
            executable='odom_compare_node',
            name='odom_compare',
            namespace='method1',
            output='screen',
            parameters=[config],
        ),
    ])
