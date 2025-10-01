import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

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

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager,
    ])
