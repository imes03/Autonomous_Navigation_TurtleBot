#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 
from launch.actions import TimerAction                  #ACW
from launch.actions import RegisterEventHandler         #ACW
from launch.event_handlers import OnProcessStart        #ACW
from launch.substitutions import Command                #ACW

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    maze_path = os.path.join(get_package_share_directory('autonomous_tb3'),'world','maze','model.sdf')
    config_dir = os.path.join(get_package_share_directory('autonomous_tb3'),'config')
    map_file = os.path.join(config_dir,'maze.yaml')
    params_file = os.path.join(config_dir,'tb3_nav_params.yaml')
    rviz_config= os.path.join(config_dir,'tb3_nav.rviz')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
  #  slam_params_file = os.path.join(get_package_share_directory('autonomous_tb3'), 'config', 'slam_toolbox_params.yaml')  #for modification of SlamToolbox

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-9.0')
    y_pose = LaunchConfiguration('y_pose', default='8.0')

    world = os.path.join(
        get_package_share_directory('autonomous_tb3'),
        'world/maze/model.sdf'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        )
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
#    Uses default turtlebot3 without camera                 #ACW : Removed to use URDF with camera
#    robot_state_publisher_cmd = IncludeLaunchDescription(
#        PythonLaunchDescriptionSource(
#            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
#        ),
#        launch_arguments={'use_sim_time': use_sim_time}.items()
#    )
#    Uses default turtlebot3 without camera                 #ACW : Removed to use URDF with camera
#    spawn_turtlebot_cmd = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource(
#            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
#        ),
#        launch_arguments={
#            'x_pose': x_pose, 'y_pose': y_pose, 'spawn_type': 'overwrite'
#        }.items()
#    )
    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'spawn_turtlebot3.launch.py'
            )
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )
   
    maze_spawner=Node(
        package='autonomous_tb3',
        output='screen',
        executable='sdf_spawner',
        name='maze_spawner',
        arguments=[maze_path,"b","0.0" ,"0.0" ]
    )


 #   robot_description = {
 #       'robot_description': Command([
 #           'xacro ',
 #           os.path.join(
 #               get_package_share_directory('turtlebot3_description'),
 #               'urdf',
 #               'turtlebot3_burger_cam.urdf'     #URDF imes03@myPC:~/ros2_turtlebot3/ws_slam/src/turtlebot3/turtlebot3_description/urdf$ 
 #
 #          )
 #       ]),
 #       'use_sim_time': True
 #   }
    robot_description = {
        'robot_description': Command([
            'xacro ',
            '/ws_slam/src/turtlebot3_simulations/turtlebot3_gazebo/urdf/turtlebot3_burger_cam.urdf'
        ]),
        'use_sim_time': True
    }
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
# # google cartographer mapping
#     maze_mapping = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#         os.path.join(get_package_share_directory('autonomous_tb3'), 'launch', 'mapping.launch.py')
#         ),
#    )

#  # SLAM toolbox mapping 

#    maze_mapping = IncludeLaunchDescription(
#        PythonLaunchDescriptionSource(
#         os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
#         ),
#    )

#    maze_mapping = Node(
 #       package='slam_toolbox',
#        executable='async_slam_toolbox_node',
#        name='slam_toolbox',
##        output='screen',
#        parameters=[{
 #           'use_sim_time': True,
 #           'base_frame': 'base_link',
 #           'odom_frame': 'odom',
#            'map_frame': 'map',
#            'scan_topic': '/scan'   
 #       }],
#        arguments=[]   
 #   )
    maze_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

# for using the custom tb3_nav_params
#     maze_mapping = Node(
#     package='slam_toolbox',
#     executable='async_slam_toolbox_node',
#     name='slam_toolbox',
#     output='screen',
#     parameters=[{
#         'use_sim_time': True,
#         'map_file_name': '',
#         'mode': 'mapping',
#         'clear_map_on_start': True  # Forces clearing old map data
#     }]
# )


    maze_nav=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/bringup_launch.py']),
        launch_arguments={
        'map': '',             # ACW: requiered  even in not use
        'params_file': params_file,
        'slam':'False',               # ACW: here we say to Nav2 to use slam instead a map  
        'use_sim_time': 'True',      # ACW: to be able to use RVIZ again
        'autostart': 'True'          # ACW: to be able to use RVIZ again
        }.items(),
     )
     

    rviz=Node(
        package='rviz2',
        output='screen',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d',rviz_config]
    )
#    gazeb  spawn_after_o = RegisterEventHandler(
#       event_handler=OnProcessStart(
#           target_action=gzserver_cmd,
#            on_start=[spawn_robot],
#       )
#   )


    maze_spawner_delayed = TimerAction(
        period=5.0,
        actions=[maze_spawner]
    )
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher)       #ACW -> removed to use modified URDF with camera
    ld.add_action(spawn_turtlebot)             #ACW -> removed to use modified URDF with camera
#    ld.add_action(robot_description_publisher)      #ACW -> Added to use modified URDF with camera
    ld.add_action(maze_spawner_delayed)
#    ld.add_action(spawn_after_gazebo)               #ACW               
    ld.add_action(maze_mapping)                     #ACW: this command enable/disable mapping
    ld.add_action(maze_nav)                         #ACW: executed before rviz
    ld.add_action(rviz)   

    return ld