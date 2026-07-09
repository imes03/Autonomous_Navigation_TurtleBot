from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_dir = os.path.join(
        get_package_share_directory('autonomous_tb3'),
        'config'
    )

    params_file = os.path.join(
        get_package_share_directory('autonomous_tb3'),
        'config',
        'tb3_nav_params.yaml'
    )

    # Nav2 bringup
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'slam': 'True',
            'use_sim_time': 'False',
            'params_file': params_file,
            'autostart': 'True'
        }.items()
    )

    return LaunchDescription([

        # Cartographer SLAM
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory',
                config_dir,
                '-configuration_basename',
                'tb3_cartographer.lua'
            ]
        ),

        # Occupancy grid publisher
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen'
        ),

        # Nav2
        nav2

    ])
