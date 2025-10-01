import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
    
    start_localization = LaunchConfiguration('start_localization')
    start_pca9685 = LaunchConfiguration('start_pca9685')

    declare_start_localization = DeclareLaunchArgument('start_localization', default_value='true', description='Start robot_localization EKF')
    declare_start_pca9685 = DeclareLaunchArgument('start_pca9685', default_value='true', description='Start PCA9685 hardware interface node')

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("caramelo_hardware"),
            "launch",
            "hardware.launch.py"
        ),
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("caramelo_controller"),
            "launch",
            "controller.launch.py"
        ),
    )

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

    def maybe_ekf(context):
        from ament_index_python.packages import PackageNotFoundError
        launch_nodes = []
        if context.perform_substitution(start_localization) == 'true':
            try:
                ekf_config = os.path.join(get_package_share_directory('caramelo_localization'), 'config', 'ekf.yaml')
                get_package_share_directory('robot_localization')
                launch_nodes.append(Node(
                    package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
                    emulate_tty=True,
                    parameters=[ ekf_config, { 'use_sim_time': LaunchConfiguration('use_sim_time') } ]
                ))
            except PackageNotFoundError:
                pass
        return launch_nodes
    ekf_group = OpaqueFunction(function=maybe_ekf)

    pca9685_node = Node(
        package='raspberrypi_bringup', executable='pca9685_controller.py', name='pca9685_controller', output='screen',
        emulate_tty=True,
        condition=IfCondition(start_pca9685)
    )

    return LaunchDescription([
        declare_start_localization,
        declare_start_pca9685,
        hardware_interface,
        controller,
        laser_driver,
        imu_driver,
        ekf_group,
        pca9685_node
    ])
