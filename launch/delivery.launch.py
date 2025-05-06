# Copyright 2023 Rodrigo Pérez-Rodríguez
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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml

def generate_launch_description():

    ld = LaunchDescription()

    # use nav2
    kobuki_pkg_dir = get_package_share_directory('kobuki')
    nav2_launch_file = os.path.join(kobuki_pkg_dir, 'launch', 'navigation.launch.py')

    kobuki_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': 'false',
            'slam': 'false',
            'map': os.path.join(pkg_dir, 'maps', 'map_uni.yaml'),
            'params_file': os.path.join(kobuki_pkg_dir, 'config', 'kobuki_nav_params.yaml'),
            'namespace': '',
            'rviz': 'True'
        }.items()
    )

    ld.add_action(kobuki_nav)

    return ld

