#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray

class RobotArmIKNode(Node):
    def __init__(self):
        super().__init__('robot_arm_ik')

        # ----------------------------------------------------------------------
        # HARDWARE LINK CONFIGURATION
        # ----------------------------------------------------------------------
        self.d  = 350.0   # Distance between motor pivot centers (mm)
        self.l1 = 275.0   # Proximal link length - Motor A side (mm)
        self.l2 = 275.0   # Proximal link length - Motor B side (mm)
        self.l3 = 350.0   # Distal link length - End effector side A (mm)
        self.l4 = 350.0   # Distal link length - End effector side B (mm)

        # Base offsets: center point is (0,0) between motors
        self.motor_a_x = -self.d / 2.0   # -175.0 mm
        self.motor_b_x =  self.d / 2.0   #  175.0 mm
        self.motor_y   = 0.0

        self.declare_parameter('command_topic', '/position_controller/commands')
        command_topic = self.get_parameter('command_topic').get_parameter_value().string_value

        self.pose_sub  = self.create_subscription(Point, '/target_pose', self.pose_callback, 10)
        self.joint_pub = self.create_publisher(Float64MultiArray, command_topic, 10)

        # Replace the hardcoded 578.10 with a declared parameter
        self.declare_parameter('y_offset_mm', 578.10)   # default keeps current behaviour
        self.y_offset = self.get_parameter('y_offset_mm').get_parameter_value().double_value

        self.get_logger().info(f"y_offset_mm = {self.y_offset}")

        self.get_logger().info(
            f"5-Bar IK ready. d={self.d} l1={self.l1} l3={self.l3} | "
            f"Publishing to {command_topic}"
        )

    def ik_5bar(self, x, y):
    # --- Motor A side (left motor, elbow bends LEFT/outward) ---
        dx_a = x - self.motor_a_x
        dy_a = y - self.motor_y
        R_a  = np.hypot(dx_a, dy_a)

        if R_a > (self.l1 + self.l3) or R_a < abs(self.l1 - self.l3) or R_a == 0:
            return None

        cos_alpha_a = (self.l1**2 + R_a**2 - self.l3**2) / (2.0 * self.l1 * R_a)
        cos_alpha_a = np.clip(cos_alpha_a, -1.0, 1.0)
        alpha_a     = np.arccos(cos_alpha_a)
        theta_a     = np.arctan2(dy_a, dx_a) + alpha_a   # elbow up-left

        # --- Motor B side (right motor, elbow bends RIGHT/outward) ---
        dx_b = x - self.motor_b_x
        dy_b = y - self.motor_y
        R_b  = np.hypot(dx_b, dy_b)

        if R_b > (self.l2 + self.l4) or R_b < abs(self.l2 - self.l4) or R_b == 0:
            return None

        cos_alpha_b = (self.l2**2 + R_b**2 - self.l4**2) / (2.0 * self.l2 * R_b)
        cos_alpha_b = np.clip(cos_alpha_b, -1.0, 1.0)
        alpha_b     = np.arccos(cos_alpha_b)
        theta_b     = np.arctan2(dy_b, dx_b) - alpha_b   # elbow up-right

        # Normalise to [-pi, pi]
        theta_a = np.arctan2(np.sin(theta_a), np.cos(theta_a))
        theta_b = np.arctan2(np.sin(theta_b), np.cos(theta_b))

        return theta_a, theta_b

    def pose_callback(self, msg):
        y = msg.y - self.y_offset
        angles = self.ik_5bar(msg.x, y)

        if angles is None:
            self.get_logger().warn(
                f"Target ({msg.x:.1f}, {msg.y:.1f}) is outside reachable workspace!"
            )
            return

        theta_a, theta_b = angles

        # Motor zero = links pointing up = 90° geometric
        # So subtract 90° (pi/2) to convert geometric angle to motor angle
        motor_a = theta_a - np.pi / 2.0
        motor_b = theta_b - np.pi / 2.0

        msg_out = Float64MultiArray()
        msg_out.data = [float(motor_a), float(motor_b)]
        self.joint_pub.publish(msg_out)

        self.get_logger().info(
            f"Target ({msg.x:.1f}, {msg.y:.1f}) mm -> "
            f"geo_a={np.degrees(theta_a):.1f}°  geo_b={np.degrees(theta_b):.1f}° | "
            f"motor_a={np.degrees(motor_a):.1f}°  motor_b={np.degrees(motor_b):.1f}°"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotArmIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()