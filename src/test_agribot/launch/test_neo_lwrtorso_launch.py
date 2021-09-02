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

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from pathlib import Path


def generate_launch_description():
    #default_world_path = os.path.join(get_package_share_directory('neo_simulation2'), 'worlds', MY_NEO_ENVIRONMENT + '.world')
    orchard_world_path = os.path.join(get_package_share_directory('orchard_gazebo'), 'worlds', 'orchardfield.world')
    print("ORCHARD PATH :" + orchard_world_path)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    #urdf = os.path.join(get_package_share_directory('neo_simulation2'), 'robots/'+MY_NEO_ROBOT+'/', MY_NEO_ROBOT+'.urdf')
    urdf_neolwr = os.path.join(get_package_share_directory('test_agribot'), 'urdf/neo_lwrtorso.urdf')
    # Use xacro command to convert urdf file to robot_description topic
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("lwr_description"),
                    "robots",
                    "lwrtorso.urdf",
                ]
            ),
            " use_sim:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_neolwr])

    #teleop =  Node(package='teleop_twist_keyboard',executable="teleop_twist_keyboard", output='screen', prefix = 'xterm -e', name='teleop')

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': orchard_world_path,
                'verbose': 'true'
            }.items()
        )
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',arguments=['-entity', 'NEO_LWRTORSO', '-topic', robot_description, '-x', '1.0', '-y' , '1.0'], output='screen')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')
    rviz2 =   Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')

    #controller
    controllers_file = os.path.join(
            get_package_share_directory('lwr_description'),
            'config',
            'lwr_gazebo_forward_controller_position.yaml'
    )

    ros2_controller = Node(
        package="controller_manager",
        executable="ros2_control_node",
        #arguments=["joint_state_broadcaster"],
        parameters=[
            robot_description,
            controllers_file],
        output="screen",
    )
    spawn_joint_taj_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen",   
    )

    spawn_forward_pos_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["forward_position_controller", "-c", "/controller_manager"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([gazebo, spawn_entity, start_robot_state_publisher_cmd, spawn_forward_pos_controller, rviz2])