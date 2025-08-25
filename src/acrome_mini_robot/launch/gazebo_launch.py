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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, LaunchConfigurationEquals

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    gz_args = LaunchConfiguration('gz_args', default='')
    slam_mode = LaunchConfiguration('slam_mode', default='mapping')  # mapping veya localization

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

    # Odometry publisher node (önceki aynı kalıyor)
    odometry_publisher_node = Node(
        package='acrome_mini_robot',
        executable='odometry_publisher.py',
        name='odometry_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        # Frame isimleri doğru olduğundan emin olmak için
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
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

    # SLAM Toolbox parametreleri
    slam_params = PathJoinSubstitution(
        [FindPackageShare('acrome_mini_robot'), 'config', 'slam_params.yaml']
    )

    # SLAM Toolbox - sync_slam_toolbox_node daha kararlı
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',  # sync kullan - daha kararlı
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params, 
            {'use_sim_time': use_sim_time},
            {'slam_mode': slam_mode}
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('/odom', '/odom')
        ]
    )

    # SLAM durumunu kontrol etme node'u (opsiyonel debug için)
    slam_status_node = Node(
        package='acrome_mini_robot',
        executable='slam_status_monitor.py',  # Bu script'i aşağıda vereceğim
        name='slam_status_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('debug_slam'))  # debug modunda çalışsın
    )

    # Navigation stack için static transform publisher (isteğe bağlı)
    # Eğer base_footprint kullanmak isterseniz
    base_footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz config
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('acrome_mini_robot'), 'rviz', 'acrome_slam_config.rviz']  # SLAM için özel config
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ROS-Gazebo bridge - Düzeltilmiş format
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock bridge
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # LaserScan bridge - doğru format
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # İsteğe bağlı diğer bridge'ler
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            # Gazebo'dan gelen topic'leri ROS topic'lerine map et
            ('/scan', '/scan'),
        ]
    )

    # TF kontrolü için debug node (opsiyonel)
    tf_debug_process = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_tools', 'view_frames'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('debug_tf'))
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

        DeclareLaunchArgument(
            'slam_mode',
            default_value='mapping',
            choices=['mapping', 'localization'],
            description='SLAM mode: mapping or localization'),

        DeclareLaunchArgument(
            'debug_slam',
            default_value='false',
            description='Enable SLAM debug monitoring'),

        DeclareLaunchArgument(
            'debug_tf',
            default_value='false',
            description='Enable TF debug output'),

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
        base_footprint_publisher,  # Static transform
        gz_spawn_entity,
        
        # Robot spawn edildikten sonra joint_state_broadcaster başlat
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[
                    LogInfo(msg="Robot spawned, starting joint state broadcaster..."),
                    joint_state_broadcaster_spawner
                ],
            )
        ),
        
        # joint_state_broadcaster başladıktan sonra odometry ve controller başlat
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    LogInfo(msg="Joint state broadcaster started, launching odometry and controllers..."),
                    odometry_publisher_node,
                    wheel_velocity_controller_spawner
                ],
            )
        ),

        # Odometry başladıktan sonra SLAM başlat (önemli sıralama!)
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wheel_velocity_controller_spawner,  # wheel_velocity_controller tamamlandıktan sonra
                on_exit=[
                    LogInfo(msg="Controllers started, launching SLAM and RViz..."),
                    # SLAM ve RViz'i aynı anda başlat
                    TimerAction(
                        period=2.0,  # 2 saniye bekle
                        actions=[
                            slam_toolbox_node,
                            rviz_node,  # RViz'i hemen başlat
                            slam_status_node  # Debug node'u da ekle
                        ]
                    )
                ],
            )
        ),

        # Debug processes (koşullu)
        tf_debug_process,
    ])