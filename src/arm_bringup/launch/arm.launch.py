import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_file = PathJoinSubstitution([
        FindPackageShare('arm_hardware_interface'),
        'urdf',
        'arm.urdf.xacro'
    ])

    robot_description_content = Command(['xacro ', urdf_file])
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

    delayed_spawner_joint_state = TimerAction(
        period=3.0,
        actions=[spawner_joint_state]
    )
    delayed_spawner_position = TimerAction(
        period=4.0,
        actions=[spawner_position]
    )

    return LaunchDescription([
        robot_state_publisher,
        control_node,
        delayed_spawner_joint_state,
        delayed_spawner_position,
    ])
