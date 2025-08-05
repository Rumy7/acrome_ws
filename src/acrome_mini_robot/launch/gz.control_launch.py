import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Gazebo related launch files
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Acrome Mini Robot Package
    pkg_acrome_mini_robot = get_package_share_directory('acrome_mini_robot')

    # Xacro and URDF processing
    robot_urdf_path = PathJoinSubstitution([
        pkg_acrome_mini_robot,
        "urdf",
        "acrome_mini_robot.urdf.xacro",
    ])
    
    robot_description_content = Command(['xacro ', robot_urdf_path])
    robot_description = {'robot_description': robot_description_content}

    # Path to the controllers YAML file
    my_controller_yaml_path = PathJoinSubstitution([
        pkg_acrome_mini_robot,
        "config",
        "my_controller.yaml",
    ])

    # Gazebo sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # ROS 2 Control Node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, my_controller_yaml_path],
        output='screen'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )
    
    # Controller Spawner
    diff_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        output='screen',
    )
    
    # Joint State Broadcaster Spawner (EKLENDİ)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,  # Önce joint state yayıncı
        diff_cont_spawner                 # Sonra kontrolcü
    ])