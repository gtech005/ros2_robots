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
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # path to the Cyclair test world
    test_world_path = os.path.join(get_package_share_directory('rovers_descriptions'), 'worlds', 'redlinescurved.world')
 
    # path to the Orchard field world
    orchard_world_path = os.path.join(get_package_share_directory('orchard_gazebo'), 'worlds', 'orchardfield.world')

    # path to the Cyclair traffic cone world
    trafficcone_world_path = os.path.join(get_package_share_directory('rovers_descriptions'), 'worlds', 'trafficConescaled.world')

    # Empty gazebo
    gazebo_default = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )
    # If want to spawn the orhcard field world
    orchard_world = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('orchard_gazebo'), 'launch'), '/spawn_orchardfield_launch.py'])
    )

    # If want to directly use the world with orchard field
    gazebo_testworld = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': test_world_path,
                'verbose': 'true'
            }.items()
        )

    # If want to use the world with traffic cones
    gazebo_trafficcone = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': trafficcone_world_path,
                'verbose': 'true'
            }.items()
        )
    
    #cyclair_urdf = os.path.join(get_package_share_directory('rovers_descriptions'), 'urdf/rover_soft.urdf')

# Use xacro command to convert urdf file to robot_description topic
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("rovers_descriptions"),
                    "urdf",
                    "cyclairbot_v2.xacro",
                ]
            ),
            " use_sim:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
   

    spawn_cyclairbot = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'cyclairbot',
                                   '-x', '1.0',
                                   '-y', '1.0',
                                   '-z', '0.5'],
                        output='screen')

    return LaunchDescription([
        #gazebo_orchard,
        gazebo_testworld, 
        #gazebo_default,
        node_robot_state_publisher,
        spawn_cyclairbot,
    ])
