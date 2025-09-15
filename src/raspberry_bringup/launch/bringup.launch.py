#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

# Bringup mínimo para o robô real (sem RViz):
# - robot_state_publisher publica robot_description e TF estática das juntas
# - ros2_control_node carrega o plugin de hardware proxy e controladores
# - spawner joint_state_broadcaster
# - spawner mecanum_controller
# - nó Python pca9685_interface converte /wheel_commands_raw -> PWM PCA9685

def generate_launch_description():
    pkg_desc = get_package_share_directory('caramelo_description')
    pkg_controller = get_package_share_directory('caramelo_controller')

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_model = LaunchConfiguration('robot_model')
    controllers_file = LaunchConfiguration('controllers_file')
    start_pca9685 = LaunchConfiguration('start_pca9685')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulated time')
    declare_robot_model = DeclareLaunchArgument('robot_model', default_value=os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro'), description='Path to robot xacro')
    declare_controllers_file = DeclareLaunchArgument('controllers_file', default_value=os.path.join(pkg_controller, 'config', 'robot_controllers.yaml'), description='ros2_control controllers YAML')
    declare_start_pca9685 = DeclareLaunchArgument('start_pca9685', default_value='true', description='Start PCA9685 hardware interface node')

    # Usa o executável xacro encontrado pelo sistema (evita depender do PATH)
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', robot_model]),
        value_type=str
    )

    # Publica robot_description e TF das juntas
    rsp_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': robot_description}]
    )

    # ros2_control principal com plugin híbrido
    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node', output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': robot_description}, controllers_file]
    )

    # Spawner dos controladores (ordem: broadcaster primeiro)
    joint_state_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )

    mecanum_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['mecanum_controller', '--controller-manager', '/controller_manager']
    )

    # Driver Python PCA9685 (converte velocidades de roda em PWM)
    pca9685_node = Node(
        package='raspberry_bringup', executable='pca9685_interface', name='pca9685_interface', output='screen',
        condition=IfCondition(start_pca9685)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_robot_model,
        declare_controllers_file,
        declare_start_pca9685,
        rsp_node,
        controller_manager,
        joint_state_spawner,
        mecanum_spawner,
        pca9685_node,
    ])
