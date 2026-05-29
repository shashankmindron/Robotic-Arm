#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import math
import struct
import time
import serial

from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayRobotState
from moveit_msgs.srv import GetStateValidity

# Import your custom mathematical brains
from five_bar_kinematics import FiveBarKinematics
from trajectory_planner import TrajectoryPlanner

class CollisionGuard:
    """A helper class to bridge the Trajectory Planner and the ROS 2 MoveIt Service"""
    def __init__(self, node_reference):
        self.node = node_reference
        
    def check_collision(self, joint_angles):
        req = GetStateValidity.Request()
        req.group_name = "arm_group"
        
        js = JointState()
        js.name = ['joint_a', 'joint_b', 'joint_distal_a', 'joint_distal_b']
        js.position = [float(joint_angles[0]), float(joint_angles[1]), 
                       float(joint_angles[2]), float(joint_angles[3])]
        req.robot_state.joint_state = js
        
        # Synchronous call (Safe here because we use a MultiThreadedExecutor)
        future = self.node.cli.call(req)
        return future.valid


class RobotArmIKNode(Node):
    def __init__(self):
        super().__init__('robot_arm_ik')

        # --- ROS 2 Multi-Threading Setup (Prevents MoveIt from freezing) ---
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.sub_cb_group = MutuallyExclusiveCallbackGroup()

        # --- MoveIt Collision Client ---
        self.cli = self.create_client(GetStateValidity, '/check_state_validity', callback_group=self.service_cb_group)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveIt Collision Service...')

        # --- Hardware Communication (UART) ---
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        try:
            self.esp32 = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Connected to ESP32 on {port} at {baud} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to open UART: {e}. Running in Simulation-Only mode!")
            self.esp32 = None

        # --- Initialize the Brains ---
        self.kinematics = FiveBarKinematics()
        self.guard = CollisionGuard(self)
        self.planner = TrajectoryPlanner(self.kinematics, self.guard)

        # --- Physical State Memory ---
        # 1. Initialize coordinate targets at the 90-degree resting pose
        self.current_x = 0.0
        self.current_y = 431.25 
        
        # FIX: Dynamically solve the initial full 4-angle tuple to prevent index errors
        self.current_geo_angles = None
        home_solutions = self.kinematics.ik_5bar(self.current_x, self.current_y)
        if home_solutions:
            for name, angles in home_solutions.items():
                # Track the specific assembly branch where proximal arms point straight up (90 deg / 1.57 rad)
                if math.isclose(angles[0], math.pi/2.0, abs_tol=0.1) and math.isclose(angles[1], math.pi/2.0, abs_tol=0.1):
                    self.current_geo_angles = angles
                    break
        
        # Hard fallback to full 4-element tuple if the solver isn't active on the first cycle
        if self.current_geo_angles is None:
            self.current_geo_angles = (math.pi / 2.0, math.pi / 2.0, -0.6478, 0.6478)

        # --- ROS 2 Publishers & Subscribers ---
        self.declare_parameter('command_topic', '/position_controller/commands')
        command_topic = self.get_parameter('command_topic').get_parameter_value().string_value

        self.pose_sub = self.create_subscription(Point, '/target_pose', self.pose_callback, 10, callback_group=self.sub_cb_group)
        self.joint_pub = self.create_publisher(Float64MultiArray, command_topic, 10)
        
        # Publisher to animate the robot in RViz
        self.display_pub = self.create_publisher(DisplayRobotState, '/display_robot_state', 10)

        self.get_logger().info("Central Nervous System Online. Ready to receive draw commands.")
        
        # --- Spawn Ghost Robot Heartbeat ---
        self.is_idle = True 
        self.startup_timer = self.create_timer(2.0, self.spawn_initial_robot, callback_group=self.sub_cb_group)

    def publish_rviz(self, geo_angles):
        """A clean helper function to update the RViz ghost robot"""
        drs = DisplayRobotState()
        js = JointState()
        js.name = ['joint_a', 'joint_b', 'joint_distal_a', 'joint_distal_b']
        
        js.position = [
            float(geo_angles[0]), 
            float(geo_angles[1]), 
            float(geo_angles[2]), 
            float(geo_angles[3])
        ]
        
        drs.state.joint_state = js
        self.display_pub.publish(drs)

    def spawn_initial_robot(self):
        """Pings the 90-degree home position to RViz until a real command is received."""
        if not self.is_idle:
            self.startup_timer.cancel() # Stop pulsing once we start drawing
            return
            
        self.publish_rviz(self.current_geo_angles)

    def pose_callback(self, msg):
        self.is_idle = False # Instantly shuts off the heartbeat timer
        
        target_x = msg.x
        target_y = msg.y 
        
        self.get_logger().info(f"--- NEW COMMAND: Drawing to Machine(X:{target_x:.1f}, Y:{target_y:.1f}) ---")

        # Generate Geometry (1mm breadcrumbs)
        waypoints = self.planner.generate_straight_line(
            self.current_x, self.current_y, 
            target_x, target_y, 
            step_mm=1.0
        )
        
        # Generate Physics (Trapezoidal Velocity)
        total_dist = math.hypot(target_x - self.current_x, target_y - self.current_y)
        
        # [TUNE THESE VALUES FOR YOUR HARDWARE]
        v_max = 100.0  # mm/s (Cruising speed)
        accel = 500.0  # mm/s^2 (Ramp up)
        decel = 500.0  # mm/s^2 (Ramp down)
        
        speeds = self.planner.generate_velocity_profile(total_dist, 1.0, v_max, accel, decel)

        # The Real-Time Execution Loop
        for i in range(1, len(waypoints)):
            wx, wy = waypoints[i]
            v_cartesian = speeds[i]
            
            # --- A. Inverse Kinematics & MoveIt Collision Guard ---
            next_geo_angles, branch = self.planner.select_best_branch(self.current_geo_angles, wx, wy)
            
            if next_geo_angles is None:
                self.get_logger().error(f"ABORTING TRAJECTORY: Collision detected at X:{wx:.1f}, Y:{wy:.1f}")
                return # Halt the entire movement instantly
                
            # --- B. Finite Difference Velocity Math ---
            dt = 1.0 / v_cartesian # Time = Distance (1mm) / Speed
            
            delta_a = next_geo_angles[0] - self.current_geo_angles[0]
            delta_b = next_geo_angles[1] - self.current_geo_angles[1]
            
            omega_a = delta_a / dt
            omega_b = delta_b / dt
            
            # Convert Radians/sec to Pulses/sec (Hz) for the T60S
            pulses_per_rad = 4000.0 / (2.0 * math.pi)
            hz_a = abs(omega_a * pulses_per_rad)
            hz_b = abs(omega_b * pulses_per_rad)
            
            # --- C. Hardware Angle Offsets ---
            motor_a = next_geo_angles[0] - (math.pi / 2.0)
            motor_b = next_geo_angles[1] - (math.pi / 2.0)

            # --- D. UART Formatting ---
            packet = struct.pack('<BffffB', 0xAA, motor_a, motor_b, hz_a, hz_b, 0xBB)
            
            # Send to ESP32
            if self.esp32 is not None:
                self.esp32.write(packet)
                
            # Publish to ROS 2 (For rqt_plot tracking)
            msg_out = Float64MultiArray()
            msg_out.data = [float(motor_a), float(motor_b)]
            self.joint_pub.publish(msg_out)

            # --- E. RViz Visualization (The Ghost Robot) ---
            self.publish_rviz(next_geo_angles)

            # --- F. State Update & Pacing ---
            self.current_x = wx
            self.current_y = wy
            self.current_geo_angles = next_geo_angles
            
            time.sleep(dt)

        self.get_logger().info(f"Arrived safely at (X:{self.current_x:.1f}, Y:{self.current_y:.1f})")


def main(args=None):
    rclpy.init(args=args)
    node = RobotArmIKNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()