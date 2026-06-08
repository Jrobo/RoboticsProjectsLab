import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory('tbot3_nav_monitor')

    params_file = os.path.join(
        package_share_dir,
        'config',
        'monitor_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='tbot3_nav_monitor',
            executable='nav_monitor_node',
            name='tbot3_nav_monitor',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='tbot3_nav_monitor',
            executable='nav_goal_metrics_node',
            name='tbot3_nav_goal_metrics',
            output='screen',
            parameters=[
                {
                    'odom_topic': '/odom',
                    'goal_topic': '/goal_pose',
                    'goal_tolerance_m': 0.25,
                    'log_directory': 'logs',
                }
            ]
        )
    ])
