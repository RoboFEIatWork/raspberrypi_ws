import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Caminho para o diretório de compartilhamento do pacote de localização
    localization_share_dir = get_package_share_directory("caramelo_localization")
    
    # Inclusão do hardware_interface
    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("caramelo_hardware"),
            "launch",
            "hardware.launch.py"
        ),
    )

    # Inclusão dos controladores
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("caramelo_controller"),
            "launch",
            "controller.launch.py"
        ),
    )

    # Nó do driver do Lidar (RPLidar)
    laser_driver = Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            parameters=[{
                'serial_port': '/dev/ttyUSB1',
                'frame_id': 'laser_frame',
                'angle_compensate': True,
            }],
            output="screen"
    )
    
    # Nó do driver da IMU (UM7)
    imu_driver = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("um7"),
            "launch",
            "um7_launch.py"
        ),
        launch_arguments={
            'port': '/dev/ttyUSB0'
        }.items()
    )

    # Nó do EKF para fusão de odometria
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(localization_share_dir, 'config', 'ekf.yaml')],
    )

    return LaunchDescription([
        hardware_interface,
        controller,
        laser_driver,
        imu_driver,
        ekf_node,
    ])
