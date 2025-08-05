from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Gazebo'nun paketleri bulması için GZ_SIM_RESOURCE_PATH ortam değişkenini ayarla
    gz_resource_path = os.path.join(
        os.getenv('HOME'),
        'acrome_ws', 'src'
        )

    urdf_path = os.path.join(
    os.getenv('HOME'),
    'acrome_ws', 'src', 'acrome_mini_robot', 'urdf', 'acrome_mini_robot.urdf'
    )

    return LaunchDescription([
        # GAZEBO_RESOURCE_PATH yerine GZ_SIM_RESOURCE_PATH kullanıldı
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gz_resource_path
        ),

         # Gazebo Harmonic başlat
        ExecuteProcess(
            cmd=['gz', 'sim', '-v4'],
            output='screen'
        ),

        # TF yayını için robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_path],
            output='screen'
        ),

        # Robot modelini spawn et
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-file', urdf_path,
                '-name', 'AcromeMiniBot'
            ],
            output='screen'
        )
    ])
