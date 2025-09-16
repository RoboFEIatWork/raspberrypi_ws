#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
    # Compute caramelo_hardware install prefix as a sibling of this package (isolated install layout)
    bringup_share = get_package_share_directory('raspberry_bringup')
    # bringup_share is .../install/raspberry_bringup/share/raspberry_bringup
    # Go three levels up to reach .../install
    install_root = os.path.dirname(os.path.dirname(os.path.dirname(bringup_share)))
    hw_prefix = os.path.join(install_root, 'caramelo_hardware')
    hw_lib = os.path.join(hw_prefix, 'lib')

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_model = LaunchConfiguration('robot_model')
    controllers_file = LaunchConfiguration('controllers_file')
    start_pca9685 = LaunchConfiguration('start_pca9685')
    # IMU always enabled (USB0). Lidar removed for now.
    start_localization = LaunchConfiguration('start_localization')
    output_mode = LaunchConfiguration('output_mode')  # screen | log

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulated time')
    declare_robot_model = DeclareLaunchArgument('robot_model', default_value=os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro'), description='Path to robot xacro')
    declare_controllers_file = DeclareLaunchArgument('controllers_file', default_value=os.path.join(pkg_controller, 'config', 'robot_controllers.yaml'), description='ros2_control controllers YAML')
    declare_start_pca9685 = DeclareLaunchArgument('start_pca9685', default_value='true', description='Start PCA9685 hardware interface node')
    # Removed start_imu and start_lidar arguments (IMU always on, lidar disabled)
    declare_start_localization = DeclareLaunchArgument('start_localization', default_value='false', description='Start robot_localization EKF (requires package installed)')
    declare_output_mode = DeclareLaunchArgument('output_mode', default_value='screen', description='Node output: screen or log')

    # Usa o executável xacro encontrado pelo sistema (sem separador manual)
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', robot_model]),
        value_type=str
    )

    # Publica robot_description e TF das juntas
    rsp_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', output=output_mode,
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': robot_description}]
    )

    # ros2_control principal com plugin híbrido
    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node', output=output_mode,
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': robot_description}, controllers_file],
        env={
            'AMENT_PREFIX_PATH': (hw_prefix + (':' + os.environ.get('AMENT_PREFIX_PATH', '') if os.environ.get('AMENT_PREFIX_PATH') else '')) if os.path.isdir(hw_prefix) else os.environ.get('AMENT_PREFIX_PATH', ''),
            'LD_LIBRARY_PATH': (hw_lib + (':' + os.environ.get('LD_LIBRARY_PATH', '') if os.environ.get('LD_LIBRARY_PATH') else '')) if os.path.isdir(hw_lib) else os.environ.get('LD_LIBRARY_PATH', ''),
            'RCUTILS_LOGGING_DIRECTORY': os.environ.get('RCUTILS_LOGGING_DIRECTORY', '/home/raspberrypi/.ros/log'),
            'HOME': os.environ.get('HOME', '/home/raspberrypi'),
        }
    )

    # Spawner dos controladores (ordem: broadcaster primeiro)
    joint_state_spawner = Node(
        package='controller_manager', executable='spawner', output=output_mode,
        emulate_tty=True,
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )

    mecanum_spawner = Node(
        package='controller_manager', executable='spawner', output=output_mode,
        emulate_tty=True,
        arguments=['mecanum_controller', '--controller-manager', '/controller_manager']
    )

    # IMU (UM6/UM7) via umx_driver on /dev/ttyUSB1
    imu_node = Node(
        package='umx_driver', executable='um6_driver', name='um6_driver', output=output_mode,
        emulate_tty=True,
        parameters=[
            { 'port': '/dev/ttyUSB0' },
            { 'use_sim_time': use_sim_time },
        ]
    )

    # RPLIDAR on /dev/ttyUSB0
    # Lidar node removed (not used currently)

    def maybe_ekf(context):
        from ament_index_python.packages import PackageNotFoundError
        launch_nodes = []
        if context.perform_substitution(start_localization) == 'true':
            try:
                ekf_config = os.path.join(get_package_share_directory('caramelo_localization'), 'config', 'ekf.yaml')
                # Ensure robot_localization package exists
                get_package_share_directory('robot_localization')
                launch_nodes.append(Node(
                    package='robot_localization', executable='ekf_node', name='ekf_filter_node', output=output_mode,
                    emulate_tty=True,
                    parameters=[ ekf_config, { 'use_sim_time': use_sim_time } ]
                ))
            except PackageNotFoundError:
                # Silent skip if not installed
                pass
        return launch_nodes
    ekf_group = OpaqueFunction(function=maybe_ekf)

    # Driver Python PCA9685 (converte velocidades de roda em PWM)
    pca9685_node = Node(
        package='raspberry_bringup', executable='pca9685_interface', name='pca9685_interface', output=output_mode,
        emulate_tty=True,
        condition=IfCondition(start_pca9685)
    )

    return LaunchDescription([
    declare_use_sim_time,
    declare_robot_model,
    declare_controllers_file,
    declare_start_pca9685,
    # IMU/Lidar args removed
    declare_start_localization,
    declare_output_mode,
        rsp_node,
        controller_manager,
        joint_state_spawner,
        mecanum_spawner,
        imu_node,
    # no lidar node
    ekf_group,
        pca9685_node,
    ])
