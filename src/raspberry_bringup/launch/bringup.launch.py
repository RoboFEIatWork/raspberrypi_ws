#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

# Bringup mínimo para o robô real:
# - robot_state_publisher publica robot_description e TF estática das juntas
# - ros2_control_node carrega o plugin de hardware proxy e controladores
# - spawner joint_state_broadcaster
# - spawner mecanum_controller
# - nó Python pca9685_interface converte /wheel_commands_raw -> PWM PCA9685

def generate_launch_description():
    pkg_desc = get_package_share_directory('caramelo_description')
    pkg_controller = get_package_share_directory('caramelo_controller')
    bringup_share = get_package_share_directory('raspberry_bringup')
    install_root = os.path.dirname(os.path.dirname(os.path.dirname(bringup_share)))
    hw_prefix = os.path.join(install_root, 'caramelo_hardware')
    hw_lib = os.path.join(hw_prefix, 'lib')

    use_sim_time = LaunchConfiguration('use_sim_time')
    # Hardcode model and controllers path to reduce arguments (single bringup style)
    robot_model_path = os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro')
    controllers_file_path = os.path.join(pkg_controller, 'config', 'robot_controllers.yaml')
    # IMU sempre ligada / PCA9685 sempre ligado / sem lidar.

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulated time')

    # Usa o executável xacro encontrado pelo sistema (sem separador manual)
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', robot_model_path]),
        value_type=str
    )

    # Publica robot_description e TF das juntas
    rsp_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': robot_description}]
    )

    # ros2_control principal com plugin híbrido
    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node', output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': robot_description}, controllers_file_path],
        env={
            'AMENT_PREFIX_PATH': (hw_prefix + (':' + os.environ.get('AMENT_PREFIX_PATH', '') if os.environ.get('AMENT_PREFIX_PATH') else '')) if os.path.isdir(hw_prefix) else os.environ.get('AMENT_PREFIX_PATH', ''),
            'LD_LIBRARY_PATH': (hw_lib + (':' + os.environ.get('LD_LIBRARY_PATH', '') if os.environ.get('LD_LIBRARY_PATH') else '')) if os.path.isdir(hw_lib) else os.environ.get('LD_LIBRARY_PATH', ''),
            'RCUTILS_LOGGING_DIRECTORY': os.environ.get('RCUTILS_LOGGING_DIRECTORY', '/home/raspberrypi/.ros/log'),
            'HOME': os.environ.get('HOME', '/home/raspberrypi'),
        }
    )

    # Spawner dos controladores (ordem: broadcaster primeiro)
    joint_state_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        emulate_tty=True,
        name='spawner_joint_state_broadcaster',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )

    mecanum_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        emulate_tty=True,
        name='spawner_mecanum_controller',
        arguments=['mecanum_controller', '--controller-manager', '/controller_manager']
    )

    # IMU (UM6/UM7) via umx_driver on /dev/ttyUSB1
    imu_node = Node(
        package='umx_driver', executable='um6_driver', name='um6_driver', output='screen',
        emulate_tty=True,
        parameters=[
            { 'port': '/dev/ttyUSB0' },
            { 'use_sim_time': use_sim_time },
        ]
    )

    # EKF sempre iniciado (falha clara se pacote ou arquivo não existir)
    ekf_config = os.path.join(get_package_share_directory('caramelo_localization'), 'config', 'ekf.yaml')
    if not os.path.exists(ekf_config):
        raise FileNotFoundError(f"EKF config ausente: {ekf_config}")
    ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
        emulate_tty=True,
        parameters=[ ekf_config, { 'use_sim_time': use_sim_time } ]
    )

    # Driver Python PCA9685 (converte velocidades de roda em PWM)
    pca9685_node = Node(
        package='raspberry_bringup', executable='pca9685_interface', name='pca9685_interface', output='screen',
        emulate_tty=True,
    )

    # Sequência determinística: joint_state -> mecanum -> EKF
    mecanum_after_js = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[mecanum_spawner]
        )
    )
    ekf_after_mecanum = RegisterEventHandler(
        OnProcessExit(
            target_action=mecanum_spawner,
            on_exit=[ekf_node]
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        rsp_node,
        controller_manager,
        imu_node,
        pca9685_node,
        joint_state_spawner,
        mecanum_after_js,
        ekf_after_mecanum,
    ])
