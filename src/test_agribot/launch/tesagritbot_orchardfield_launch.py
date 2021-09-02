# Copyright 2020 Open Source Robotics Foundation, Inc.
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
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
#from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Empty gazebo
    gazebo_default = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )
    #Start the orhcard field world
    orchard_world = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('orchard_gazebo'), 'launch'), '/spawn_orchardfield_launch.py'])
    )


    diffbot_path = os.path.join(
        get_package_share_directory('diffbot_description'))

    xacro_file = os.path.join(diffbot_path,
                              'urdf',
                              'diffbot_system.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
   

    spawn_diffbot = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'diffbot',
                                   '-x', '1.0',
                                   '-y', '1.0',
                                   '-z', '1.0'],
                        output='screen')

    return LaunchDescription([
        #orchard_world,
        gazebo_default,
        node_robot_state_publisher,
        spawn_diffbot,
    ])
