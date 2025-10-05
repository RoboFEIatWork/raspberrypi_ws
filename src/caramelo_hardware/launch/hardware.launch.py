import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    use_mpu = LaunchConfiguration("use_mpu")

    use_mpu_arg = DeclareLaunchArgument(
        "use_mpu",
        default_value="true",
        description="Use MPU6050 IMU driver if true, otherwise use UM6 driver"
    )


    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("caramelo_description"),
                    "urdf",
                    "robot.urdf.xacro",
                ),
                " is_sim:=False"
            ]
        ),
        value_type=str,
    )

    # Nó do Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False
        }]
    )

    # Nó do Controller Manager (ros2_control)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description,
            "use_sim_time": False},
            os.path.join(
                get_package_share_directory("caramelo_controller"),
                "config",
                "robot_controllers.yaml"
            )
        ]
    )


    # Node for MPU6050 driver (only if use_mpu is true)
    mpu6050_driver_node = Node(
        package="caramelo_hardware",
        executable="mpu6050_driver.py",
        condition=IfCondition(use_mpu),
        output="screen"
    )

    # Node for UM6 driver (only if use_mpu is false)
    um6_driver_node = Node(
        package="umx_driver",
        executable="um6_driver",
        name="um6_driver",
        parameters=[{'port': '/dev/ttyUSB0'}],
        condition=UnlessCondition(use_mpu),
        output="screen"
    )

    return LaunchDescription([
        use_mpu_arg,
        robot_state_publisher_node,
        controller_manager,
        mpu6050_driver_node,
        um6_driver_node,
    ])
