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

        self.get_logger().info(
            f"5-Bar IK ready. d={self.d} l1={self.l1} l3={self.l3} | "
            f"Publishing to {command_topic}"
        )

    def ik_5bar(self, x, y):
        """
        Closed-form IK for symmetric 5-bar parallel linkage.
        Returns (theta_a, theta_b) in radians, or None if unreachable.
        """
        # --- Motor A side ---
        dx_a = x - self.motor_a_x
        dy_a = y - self.motor_y
        R_a  = np.hypot(dx_a, dy_a)

        if R_a > (self.l1 + self.l3) or R_a < abs(self.l1 - self.l3) or R_a == 0:
            return None

        cos_alpha_a = (self.l1**2 + R_a**2 - self.l3**2) / (2.0 * self.l1 * R_a)
        cos_alpha_a = np.clip(cos_alpha_a, -1.0, 1.0)
        alpha_a     = np.arccos(cos_alpha_a)
        theta_a     = np.arctan2(dy_a, dx_a) + alpha_a

        # --- Motor B side ---
        dx_b = x - self.motor_b_x
        dy_b = y - self.motor_y
        R_b  = np.hypot(dx_b, dy_b)

        if R_b > (self.l2 + self.l4) or R_b < abs(self.l2 - self.l4) or R_b == 0:
            return None

        cos_alpha_b = (self.l2**2 + R_b**2 - self.l4**2) / (2.0 * self.l2 * R_b)
        cos_alpha_b = np.clip(cos_alpha_b, -1.0, 1.0)
        alpha_b     = np.arccos(cos_alpha_b)
        theta_b     = np.arctan2(dy_b, dx_b) - alpha_b

        # Normalise to [-pi, pi]
        theta_a = np.arctan2(np.sin(theta_a), np.cos(theta_a))
        theta_b = np.arctan2(np.sin(theta_b), np.cos(theta_b))

        return theta_a, theta_b

    def pose_callback(self, msg):
        y=msg.y-578.10
        angles = self.ik_5bar(msg.x, y)

        if angles is None:
            self.get_logger().warn(
                f"Target ({msg.x:.1f}, {msg.y:.1f}) is outside reachable workspace!"
            )
            return

        theta_a, theta_b = angles
        msg_out = Float64MultiArray()
        msg_out.data = [float(theta_a), float(theta_b)]
        self.joint_pub.publish(msg_out)

        self.get_logger().info(
            f"Target ({msg.x:.1f}, {msg.y:.1f}) mm -> "
            f"joint_a={np.degrees(theta_a):.1f}°  joint_b={np.degrees(theta_b):.1f}°"
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