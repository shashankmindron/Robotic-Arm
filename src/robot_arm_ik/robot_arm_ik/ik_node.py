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
import threading

from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayRobotState

# Import your custom mathematical brains
from robot_arm_ik.five_bar_kinematics import FiveBarKinematics
from robot_arm_ik.astar_planner import AStarPlanner

class RobotArmIKNode(Node):
    def __init__(self, planner_node):
        super().__init__('robot_arm_ik')

        self.sub_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.telemetry_cb_group = MutuallyExclusiveCallbackGroup()

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        try:
            self.esp32 = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Connected to ESP32 on {port} at {baud} baud.")
        except Exception as e:
            self.get_logger().warn(f"Failed to open UART: {e}. Running in Simulation-Only mode!")
            self.esp32 = None

        self.hardware_alarm = False
        self.live_rad_a = math.pi / 2.0  
        self.live_rad_b = math.pi / 2.0
        self.is_running = True
        
        if self.esp32 is not None:
            self.listener_thread = threading.Thread(target=self.serial_listener, daemon=True)
            self.listener_thread.start()

        self.kinematics = FiveBarKinematics()
        self.planner = planner_node 

        self.current_x = 0.0
        self.current_y = 431.25 
        self.current_geo_angles = None
        self.current_branch = None 
        
        home_solutions = self.kinematics.ik_5bar(self.current_x, self.current_y)
        if home_solutions:
            for name, angles in home_solutions.items():
                if math.isclose(angles[0], math.pi/2.0, abs_tol=0.1) and math.isclose(angles[1], math.pi/2.0, abs_tol=0.1):
                    self.current_geo_angles = angles
                    self.current_branch = name
                    break
        
        if self.current_geo_angles is None:
            self.current_geo_angles = (math.pi / 2.0, math.pi / 2.0, -0.6478, 0.6478)
            self.current_branch = 'out_out'

        # =========================================================================
        # THE FIX: CONTINUOUS MOTOR TRACKERS
        # This isolates the physical motors from the mathematical Wrap-Around bug!
        # =========================================================================
        self.continuous_motor_a = self.current_geo_angles[0] - (math.pi / 2.0)
        self.continuous_motor_b = self.current_geo_angles[1] - (math.pi / 2.0)

        self.execution_queue = []
        self.exec_timer = self.create_timer(0.01, self.execution_timer_callback, callback_group=self.timer_cb_group)
        self.telemetry_timer = self.create_timer(0.02, self.broadcast_telemetry_callback, callback_group=self.telemetry_cb_group)

        self.declare_parameter('command_topic', '/position_controller/commands')
        command_topic = self.get_parameter('command_topic').get_parameter_value().string_value

        self.pose_sub = self.create_subscription(Point, '/target_pose', self.pose_callback, 10, callback_group=self.sub_cb_group)
        self.joint_pub = self.create_publisher(Float64MultiArray, command_topic, 10)
        self.display_pub = self.create_publisher(DisplayRobotState, '/display_robot_state', 10)
        self.live_joint_pub = self.create_publisher(JointState, '/live_joint_states', 10)

        self.get_logger().info(f"Autonomous Nervous System Online. Layer: '{self.current_branch}'")
        self.is_idle = True 
        self.startup_timer = self.create_timer(2.0, self.spawn_initial_robot, callback_group=self.sub_cb_group)

    def serial_listener(self):
        while self.is_running:
            try:
                if self.esp32.in_waiting >= 11:
                    byte = self.esp32.read(1)
                    if byte == b'\xCC':
                        payload = self.esp32.read(10)
                        if payload[-1] == 0xDD:
                            status, raw_rad_a, raw_rad_b = struct.unpack('<Bff', payload[:9])
                            self.live_rad_a = raw_rad_a + (math.pi / 2.0)
                            self.live_rad_b = raw_rad_b + (math.pi / 2.0)
                            
                            if status == 1 and not self.hardware_alarm:
                                self.hardware_alarm = True
                                self.get_logger().fatal("HARDWARE ALARM DETECTED! Clearing queue.")
                                self.execution_queue.clear()
            except Exception:
                break

    def broadcast_telemetry_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_a', 'joint_b']
        msg.position = [float(self.live_rad_a), float(self.live_rad_b)]
        self.live_joint_pub.publish(msg)

    def destroy_node(self):
        self.is_running = False
        if self.esp32 is not None:
            time.sleep(0.1)
            self.esp32.close()
        super().destroy_node()

    def publish_rviz(self, geo_angles):
        drs = DisplayRobotState()
        js = JointState()
        js.name = ['joint_a', 'joint_b', 'joint_distal_a', 'joint_distal_b']
        js.position = [float(geo_angles[0]), float(geo_angles[1]), float(geo_angles[2]), float(geo_angles[3])]
        drs.state.joint_state = js
        self.display_pub.publish(drs)

    def spawn_initial_robot(self):
        if not self.is_idle:
            self.startup_timer.cancel()
            return
        self.publish_rviz(self.current_geo_angles)

    def execution_timer_callback(self):
        if self.hardware_alarm or not self.execution_queue:
            return

        cmd = self.execution_queue.pop(0)
        
        if self.esp32 is not None:
            packet = struct.pack('<BffffB', 0xAA, cmd['motor_a'], cmd['motor_b'], cmd['hz_a'], cmd['hz_b'], 0xBB)
            self.esp32.write(packet)
            
        msg_out = Float64MultiArray()
        msg_out.data = [float(cmd['motor_a']), float(cmd['motor_b'])]
        self.joint_pub.publish(msg_out)
        self.publish_rviz(cmd['geo_angles'])
        
        if not self.execution_queue:
            self.get_logger().info("--- Route Complete! Constant Velocity Maintained. ---")

    def pose_callback(self, msg):
        if self.hardware_alarm:
            self.get_logger().error("Command Rejected: System is locked in ALARM state.")
            return

        self.is_idle = False
        target_x = msg.x
        target_y = msg.y 
        
        self.get_logger().info(f"\n[PLANNER] Received Target (X:{target_x:.1f}, Y:{target_y:.1f})")
        
        trajectory = self.planner.plan_trajectory(
            self.current_x, self.current_y, self.current_branch, target_x, target_y
        )
        
        if not trajectory:
            self.get_logger().error("Execution Aborted: No safe route found.")
            return

        v_max = 150.0  
        pulses_per_rad = 3200.0 / (2.0 * math.pi)

        for step in trajectory:
            if step['action'] == 'ik_move':
                wx = step['target']['x']
                wy = step['target']['y']
                branch = step['locked_branch']
                
                step_dist = math.hypot(wx - self.current_x, wy - self.current_y)
                if step_dist < 0.1: continue
                
                segment_duration = step_dist / v_max
                num_ticks = max(1, int(segment_duration / 0.01))
                
                start_x = self.current_x
                start_y = self.current_y
                
                for tick in range(1, num_ticks + 1):
                    t = tick / num_ticks
                    micro_x = start_x + t * (wx - start_x)
                    micro_y = start_y + t * (wy - start_y)
                    
                    next_geo_angles = self.kinematics.ik_5bar(micro_x, micro_y, filter_singularities=False)[branch]
                    
                    # --- SHORTEST PATH TRACKER FOR MOVEMENT ---
                    target_a = next_geo_angles[0] - (math.pi / 2.0)
                    target_b = next_geo_angles[1] - (math.pi / 2.0)
                    
                    diff_a = target_a - self.continuous_motor_a
                    diff_b = target_b - self.continuous_motor_b
                    
                    delta_a = math.atan2(math.sin(diff_a), math.cos(diff_a))
                    delta_b = math.atan2(math.sin(diff_b), math.cos(diff_b))
                    
                    self.continuous_motor_a += delta_a
                    self.continuous_motor_b += delta_b
                    
                    omega_a = delta_a / 0.01
                    omega_b = delta_b / 0.01
                    
                    hz_a = abs(omega_a * pulses_per_rad)
                    hz_b = abs(omega_b * pulses_per_rad)

                    self.execution_queue.append({
                        'motor_a': self.continuous_motor_a, 
                        'motor_b': self.continuous_motor_b,
                        'hz_a': hz_a, 'hz_b': hz_b,
                        'geo_angles': next_geo_angles
                    })

                    self.current_x = micro_x
                    self.current_y = micro_y
                    self.current_geo_angles = next_geo_angles
                    
                self.current_branch = branch
                
            elif step['action'] == 'fk_portal_flip':
                new_branch = step['new_branch']
                
                target_geo_angles = self.kinematics.ik_5bar(self.current_x, self.current_y, filter_singularities=False)[new_branch]
                
                flip_duration = 1.0  
                flip_steps = 100     
                
                # --- SHORTEST PATH TRACKER FOR FLIPS ---
                target_a = target_geo_angles[0] - (math.pi / 2.0)
                target_b = target_geo_angles[1] - (math.pi / 2.0)
                
                diff_a = target_a - self.continuous_motor_a
                diff_b = target_b - self.continuous_motor_b
                
                total_delta_a = math.atan2(math.sin(diff_a), math.cos(diff_a))
                total_delta_b = math.atan2(math.sin(diff_b), math.cos(diff_b))
                
                omega_a = total_delta_a / flip_duration
                omega_b = total_delta_b / flip_duration
                
                hz_a = abs(omega_a * pulses_per_rad)
                hz_b = abs(omega_b * pulses_per_rad)

                frame_delta_a = total_delta_a / flip_steps
                frame_delta_b = total_delta_b / flip_steps

                start_angles = self.current_geo_angles

                for frame in range(1, flip_steps + 1):
                    # Tick the continuous physical hardware trackers safely forward
                    self.continuous_motor_a += frame_delta_a
                    self.continuous_motor_b += frame_delta_b
                    
                    # Recalculate wrapped angles purely so RViz looks correct
                    interp_a = start_angles[0] + (frame * (target_geo_angles[0] - start_angles[0]) / flip_steps)
                    interp_b = start_angles[1] + (frame * (target_geo_angles[1] - start_angles[1]) / flip_steps)
                    
                    elbow_a_x = self.kinematics.motor_a_x + self.kinematics.l1 * math.cos(interp_a)
                    elbow_a_y = self.kinematics.motor_y   + self.kinematics.l1 * math.sin(interp_a)
                    elbow_b_x = self.kinematics.motor_b_x + self.kinematics.l2 * math.cos(interp_b)
                    elbow_b_y = self.kinematics.motor_y   + self.kinematics.l2 * math.sin(interp_b)
                    
                    abs_da = math.atan2(self.current_y - elbow_a_y, self.current_x - elbow_a_x)
                    abs_db = math.atan2(self.current_y - elbow_b_y, self.current_x - elbow_b_x)
                    
                    distal_a = math.atan2(math.sin(abs_da - interp_a), math.cos(abs_da - interp_a))
                    distal_b = math.atan2(math.sin(abs_db - interp_b), math.cos(abs_db - interp_b))

                    self.execution_queue.append({
                        'motor_a': self.continuous_motor_a, 
                        'motor_b': self.continuous_motor_b,
                        'hz_a': max(hz_a, 10.0), 'hz_b': max(hz_b, 10.0),
                        'geo_angles': (interp_a, interp_b, distal_a, distal_b)
                    })

                self.current_geo_angles = target_geo_angles
                self.current_branch = new_branch
                
        self.get_logger().info(f"Buffered {len(self.execution_queue)} synchronized frames. Executing...")


def main(args=None):
    rclpy.init(args=args)
    planner_node = AStarPlanner(step_size=15.0) 
    ik_node = RobotArmIKNode(planner_node)
    
    executor = MultiThreadedExecutor()
    executor.add_node(planner_node)
    executor.add_node(ik_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ik_node.destroy_node()
        planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()