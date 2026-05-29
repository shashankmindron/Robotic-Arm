import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue   # <-- the fix


def generate_launch_description():

    urdf_file = PathJoinSubstitution([
        FindPackageShare('arm_hardware_interface'),
        'urdf',
        'arm.urdf.xacro'
    ])

    # Wrap in ParameterValue(..., value_type=str) so ROS 2 treats the
    # xacro output as a plain string, not YAML.
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    robot_description = {'robot_description': robot_description_content}

    controller_params = PathJoinSubstitution([
        FindPackageShare('arm_hardware_interface'),
        'config',
        'ros2_controllers.yaml'
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, controller_params],
    )

    spawner_joint_state = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen'
    )

    spawner_position = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller', '-c', '/controller_manager'],
        output='screen'
    )

    ik_node = Node(
        package='robot_arm_ik',
        executable='ik_node',
        output='screen',
        parameters=[{'y_offset_mm': 578.10}]
    )

    delayed_spawner_joint_state = TimerAction(
        period=3.0,
        actions=[spawner_joint_state]
    )
    delayed_spawner_position = TimerAction(
        period=4.0,
        actions=[spawner_position]
    )
    delayed_ik_node = TimerAction(
        period=6.0,
        actions=[ik_node]
    )

    return LaunchDescription([
        robot_state_publisher,
        control_node,
        delayed_spawner_joint_state,
        delayed_spawner_position,
        delayed_ik_node,
    ])