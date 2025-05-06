# Copyright 2024 Juan Carlos Manzanares Serrano
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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():

    navigation_system_dir = get_package_share_directory('navigation_system')
    nav2_dir = get_package_share_directory('nav2_bringup')
    robocup_dir = get_package_share_directory('robocup_bringup')
    slam_dir = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('params_file')
    nav_mode = LaunchConfiguration('mode')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='False')

    declare_map_cmd = DeclareLaunchArgument(
        'map', default_value=os.path.join(
            robocup_dir,
            'maps',
            'lab_robocup.yaml')
    )

    declare_nav_params_cmd = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(
            robocup_dir,
            'params',
            'tiago_nav_params.yaml')
    )

    declare_slam_params_cmd = DeclareLaunchArgument(
        'slam_params_file', default_value=os.path.join(
            robocup_dir,
            'params',
            'tiago_nav_follow_params.yaml')
    )
    declare_nav_mode_cmd = DeclareLaunchArgument(
        'mode', default_value='amcl')


    lifecycle_nodes = ['map_server',
                       'amcl',
                       'slam_toolbox',
                       'controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother'
                       ]

    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_system_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': params_file,
            'rviz': rviz
        }.items()
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_system_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': slam_params_file,
            'autostart': 'false',
            'use_lifecycle_manager': 'true'
        }.items()
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz
        }.items(),
        condition=IfCondition(rviz)
    )

    navigation_system_node = Node(
        package='navigation_system',
        executable='navigation_system_node',
        name='navigation_system_node',
        output='screen',
        parameters=[{'nodes': lifecycle_nodes,
                     'mode': nav_mode}]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_nav_params_cmd)
    ld.add_action(declare_nav_mode_cmd)
    ld.add_action(declare_slam_params_cmd)
    ld.add_action(localization_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(slam_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(navigation_system_node)

    return ld
