import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_desc = get_package_share_directory('caramelo_description')
    pkg_controller = get_package_share_directory('caramelo_controller')

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_model = LaunchConfiguration('robot_model')
    controllers_file = LaunchConfiguration('controllers_file')
    output_mode = LaunchConfiguration('output_mode')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulated time')
    declare_robot_model = DeclareLaunchArgument('robot_model', default_value=os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro'), description='Path to robot xacro')
    declare_controllers_file = DeclareLaunchArgument('controllers_file', default_value=os.path.join(pkg_controller, 'config', 'robot_controllers.yaml'), description='ros2_control controllers YAML')
    declare_output_mode = DeclareLaunchArgument('output_mode', default_value='screen', description='Node output: screen or log')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', robot_model]),
        value_type=str
    )

    rsp_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', output=output_mode,
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': robot_description}]
    )

    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node', output=output_mode,
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': robot_description}, controllers_file],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_robot_model,
        declare_controllers_file,
        declare_output_mode,
        rsp_node,
        controller_manager,
    ])
