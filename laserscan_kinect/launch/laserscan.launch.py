# Copyright 2016-2021 Michał Drwięga (drwiega.michal@gmail.com)
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
import launch
from launch_ros.actions import Node


def generate_launch_description():
    ld = launch.LaunchDescription()

    config = os.path.join(
        get_package_share_directory('laserscan_kinect'),
        'config',
        'params.yaml'
    )

    laserscan_node = Node(
        package='laserscan_kinect',
        node_executable='laserscan_kinect_exe',
        parameters=[config],
        remappings=[
            ('/image', '/camera/depth/image_raw'),
            ('/camera_info', '/camera/depth/camera_info'),
        ]
    )
    ld.add_action(laserscan_node)

    return ld
