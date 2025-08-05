from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Controller yolu
    controller_config = os.path.join(
        get_package_share_directory('acrome_mini_robot'),
        'config',
        'my_controller.yaml'
    )

    return LaunchDescription([
        # Gazebo Başlatıcı
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so'],
            output='screen'
        ),

        # Controllerlar
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),

        # Teleop Keyboard
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')]
        )
    ])