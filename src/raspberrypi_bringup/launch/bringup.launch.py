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
        ekf_node,
    ])

