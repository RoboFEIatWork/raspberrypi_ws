import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # ========== PACOTES E ARQUIVOS ==========
    pkg_caramelo_hardware = get_package_share_directory("caramelo_hardware")
    pkg_caramelo_controller = get_package_share_directory("caramelo_controller")
    pkg_caramelo_localization = get_package_share_directory("caramelo_localization")
    
    # ========== COMPONENTES DO ROBÔ ==========

    # 1. Inicia o nó do ros2_control (ControllerManager) e o robot_state_publisher
    hardware_interface = IncludeLaunchDescription(
        os.path.join(pkg_caramelo_hardware, "launch", "hardware.launch.py"),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # 2. Inicia os spawners para os controladores (mecanum_controller e joint_state_broadcaster)
    controller = IncludeLaunchDescription(
        os.path.join(pkg_caramelo_controller, "launch", "controller.launch.py"),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # 3. Driver do Lidar (RPLidar S2)
    laser_driver_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': 1000000,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'DenseBoost',
            'use_sim_time': False,
        }],
        output="screen"
    )
    
    # 4. Driver do IMU (UM6)
    imu_driver_node = Node(
        package="umx_driver", # O pacote no workspace é 'umx_driver'
        executable="um6_driver", # O executável que você mencionou
        name="um7_driver",
        parameters=[{'port': '/dev/ttyUSB0'}],
        output="screen"
    )

    # 5. Nó de interface com o hardware PCA9685
    pca9685_node = Node(
        package='raspberrypi_bringup', 
        executable='pca9685_controller.py', 
        name='pca9685_controller', 
        output='screen',
        emulate_tty=True,
    )

    # 6. EKF para fusão de odometria e IMU
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_caramelo_localization, 'config', 'ekf.yaml'),
            {'use_sim_time': False}
        ],
    )

    return LaunchDescription([
        hardware_interface,
        controller,
        laser_driver_node,
        imu_driver_node,
        pca9685_node,
        ekf_node,
    ])

