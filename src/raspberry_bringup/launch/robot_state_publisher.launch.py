#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
	pkg_desc = get_package_share_directory('caramelo_description')
	pkg_bringup = get_package_share_directory('raspberry_bringup')
	pkg_controller = get_package_share_directory('caramelo_controller')

	# Args
	use_sim_time = LaunchConfiguration('use_sim_time')
	robot_model = LaunchConfiguration('robot_model')
	controllers_file = LaunchConfiguration('controllers_file')
	start_rviz = LaunchConfiguration('start_rviz')
	use_js_broadcaster = LaunchConfiguration('use_js_broadcaster')

	declare_use_sim_time = DeclareLaunchArgument(
		'use_sim_time', default_value='false',
		description='Use simulated time')

	declare_robot_model = DeclareLaunchArgument(
		'robot_model',
		default_value=os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro'),
		description='Path to robot xacro')

	declare_controllers_file = DeclareLaunchArgument(
		'controllers_file',
		default_value=os.path.join(pkg_controller, 'config', 'robot_controllers.yaml'),
		description='Path to ros2_control controllers YAML')

	declare_start_rviz = DeclareLaunchArgument(
		'start_rviz', default_value='true',
		description='Start RViz to visualize robot_description')
	declare_use_js_broadcaster = DeclareLaunchArgument(
		'use_js_broadcaster', default_value='false',
		description='Spawn joint_state_broadcaster (false when using encoder-based JointState)')

	robot_description = ParameterValue(Command(['xacro ', robot_model]), value_type=str)

	# robot_state_publisher
	rsp_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		output='screen',
		parameters=[
			{'use_sim_time': use_sim_time},
			{'robot_description': robot_description},
		]
	)

	# controller manager
	controller_manager = Node(
		package='controller_manager',
		executable='ros2_control_node',
		parameters=[
			{'use_sim_time': use_sim_time},
			{'robot_description': robot_description},
			controllers_file,
		],
		output='screen'
	)

	# Spawners
	joint_state_spawner = Node(
		package='controller_manager',
		executable='spawner',
		arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
		condition=IfCondition(use_js_broadcaster)
	)

	mecanum_spawner = Node(
		package='controller_manager',
		executable='spawner',
		arguments=['mecanum_controller', '--controller-manager', '/controller_manager']
	)

	# Optional RViz
	rviz_node = Node(
		package='rviz2', executable='rviz2', name='rviz2', output='screen',
		arguments=['-d', os.path.join(pkg_desc, 'rviz', 'caramelo.rviz')],
		condition=IfCondition(start_rviz)
	)

	odom_node = Node(
		package='raspberry_bringup', executable='odometry_node', name='odometry_node', output='screen',
		parameters=[
			{'cpr': 2048}, {'publish_rate': 50.0}, {'wheel_radius': 0.05}, {'lx_plus_ly': 0.39},
			{'base_frame_id': 'base_footprint'}, {'odom_frame_id': 'odom'}, {'publish_tf': True},
			{'backend': 'auto'},
			{'fl_a': 17}, {'fl_b': 27}, {'fl_invert': False},
			{'fr_a': 22}, {'fr_b': 23}, {'fr_invert': False},
			{'bl_a': 24}, {'bl_b': 25}, {'bl_invert': False},
			{'br_a': 5}, {'br_b': 6}, {'br_invert': False},
		]
	)

	return LaunchDescription([
		declare_use_sim_time,
		declare_robot_model,
		declare_controllers_file,
		declare_start_rviz,
		declare_use_js_broadcaster,
		controller_manager,
		joint_state_spawner,
		mecanum_spawner,
		rsp_node,
		rviz_node,
		odom_node,
	])
