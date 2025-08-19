# Copyright 2021 Open Source Robotics Foundation, Inc.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    gz_args = LaunchConfiguration('gz_args', default='')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('acrome_mini_robot'),
                 'urdf', 'acrome_mini_robot.xacro']
            ),
        ]
    )
    robot_description = {
    'robot_description': ParameterValue(
        robot_description_content,
        value_type=str)
    }
    
    # Controller config
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('acrome_mini_robot'),
            'config',
            'acrome_controller.yaml',
        ]
    )

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Spawn robot in Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'acrome_mini_robot', '-allow_renaming', 'true'],
    )

    # joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Tek grup velocity controller (3 teker birden)
    wheel_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'wheel_velocity_controller',
            '--param-file',
            robot_controllers,
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # SLAM Toolbox
    slam_params = PathJoinSubstitution(
        [FindPackageShare('acrome_mini_robot'), 'config', 'slam_params.yaml']
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',   # mapping için sync/online_async ikisi de olur
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}]
    )

    # RViz aç
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('acrome_mini_robot'), 'rviz', 'acrome_config.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='If true, use simulated clock'),
            
        DeclareLaunchArgument(
            'gz_args',
            default_value='',
            description='Arguments for Gazebo'),

        # Launch gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [gz_args, ' -r -v 1 /home/halit/acrome_ws/src/acrome_mini_robot/worlds/acrome_mini_robot.world'])]),

        # Nodes
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        
        # Robot spawn edildikten sonra joint_state_broadcaster başlat
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        
        # joint_state_broadcaster başladıktan sonra diğer controller'lar başlat
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[wheel_velocity_controller_spawner],
            )
        ),
        
        # SLAM ve RViz'i biraz gecikmeli başlat
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wheel_velocity_controller_spawner,
                on_exit=[slam_toolbox_node, rviz_node],
            )
        ),
    ])