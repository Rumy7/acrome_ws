import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- Paket Yollarını Tanımlama ---
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_acrome_mini_robot = get_package_share_directory('acrome_mini_robot')

    # XACRO dosyasının tam yolunu oluşturma
    robot_urdf_path = PathJoinSubstitution([
        pkg_acrome_mini_robot,
        "urdf",
        "acrome_mini_robot.urdf.xacro",
    ])

    # HATA DÜZELTİLDİ: robot_description'ı string olarak iletmek için ParameterValue kullanıldı
    robot_description_content = ParameterValue(Command(['xacro ', robot_urdf_path]), value_type=str)
    robot_description = {'robot_description': robot_description_content}

    # Gazebo'nun model ve eklentileri bulması için ortam değişkenini ayarla
    gz_resource_path = os.path.join(os.getenv('HOME'), 'acrome_ws', 'src')

    # --- Başlatma Eylemleri ---

    # 1. Gazebo simülasyonunu başlat
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 2. Robot Durum Yayıncısı (Robot State Publisher)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # 3. Robotu Gazebo'ya spawn etme
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description',
                   '-name', 'AcromeMiniBot'],
        output='screen',
    )

    # 4. Ortak Durum Yayıncısını (Joint State Broadcaster) Başlat
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # 5. Diferansiyel Sürücü Kontrolcüsünü Başlat
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # 6. Teleop Düğümünü Başlat
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel')],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gz_resource_path
        ),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        teleop_node,
    ])
