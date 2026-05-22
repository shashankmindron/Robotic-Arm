import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command  # <-- Uses the system tool directly

def generate_launch_description():
    # Locate files
    pkg_share = FindPackageShare(package='arm_hardware_interface').find('arm_hardware_interface')
    xacro_file = os.path.join(pkg_share, 'urdf', 'arm.urdf.xacro')
    controllers_file = os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')

    # Run xacro via command-line execution instead of Python import
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # Node 1: Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Node 2: Controller Manager (The core ros2_control node)
    node_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file],
        output='screen'
    )

    # Spawner 1: Joint State Broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Spawner 2: Arm Position Controller (Starts AFTER broadcaster is up)
    spawn_arm_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_position_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Delay loading the position controller until the broadcaster is live
    delay_position_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_arm_position_controller],
        )
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_controller_manager,
        spawn_joint_state_broadcaster,
        delay_position_controller
    ])