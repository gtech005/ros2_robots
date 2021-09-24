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
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_share = FindPackageShare(package='rovers_descriptions').find('rovers_descriptions')
    world_file_name = 'redlinescurved.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    world = LaunchConfiguration('world')
    declare_world_cmd = DeclareLaunchArgument(
                            name='world',
                            default_value=world_path,
                            description='Full path to the world model file to load')
    # Empty gazebo
    gazebo_default = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': world}.items()
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    
    #cyclair_urdf = os.path.join(get_package_share_directory('rovers_descriptions'), 'urdf/rover_soft.urdf'))

# Use xacro command to convert urdf file to robot_description topic
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("rovers_descriptions"),
                    "urdf",
                    "cyclairbot_castorv1.xacro",
                ]
            ),
            #" use_sim_time:=true",
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
                                   '-x', '0.0',
                                   '-y', '0.0',
                                   '-z', '0.5'],
                        output='screen')

    return LaunchDescription([
        declare_world_cmd,
        gazebo_default,
        node_robot_state_publisher,
        spawn_cyclairbot,
    ])
