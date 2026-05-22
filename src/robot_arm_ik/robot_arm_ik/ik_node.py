#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

class RobotArmIKNode(Node):
    def __init__(self):
        super().__init__('robot_arm_ik')
        
        # ----------------------------------------------------------------------
        # HARDWARE LINK CONFIGURATION (Update these with your finalized CAD values)
        # ----------------------------------------------------------------------
        self.d = 80.0    # Distance between motor pivot centers (mm)
        self.l1 = 120.0  # Proximal link length - Motor A side (mm)
        self.l2 = 120.0  # Proximal link length - Motor B side (mm)
        self.l3 = 150.0  # Distal link length - End effector side A (mm)
        self.l4 = 150.0  # Distal link length - End effector side B (mm)

        # Base offsets for motor configurations: center point is (0,0) between motors
        self.motor_a_x = -self.d / 2.0
        self.motor_b_x = self.d / 2.0
        self.motor_y = 0.0

        # Subscriptions and Publications
        self.pose_sub = self.create_subscription(Point, '/target_pose', self.pose_callback, 10)
        self.joint_pub = self.create_publisher(Float32MultiArray, '/joint_targets', 10) 

        self.get_logger().info("Parallel 5-Bar Linkage IK Node initialized and ready.")

    def ik_5bar(self, x, y):
        """
        Computes joint angles theta_a and theta_b (in radians) for the given (x, y).
        Returns (theta_a, theta_b) or None if point is physically unreachable.
        """
        # --- Solve Motor A side ---
        dx_a = x - self.motor_a_x
        dy_a = y - self.motor_y
        R_a = np.hypot(dx_a, dy_a)

        # Check reachability triangle limits for Side A
        if R_a > (self.l1 + self.l3) or R_a < abs(self.l1 - self.l3) or R_a == 0:
            return None

        # Law of cosines for angle inside joint space triangle
        cos_alpha_a = (self.l1**2 + R_a**2 - self.l3**2) / (2.0 * self.l1 * R_a)
        cos_alpha_a = np.clip(cos_alpha_a, -1.0, 1.0)
        alpha_a = np.arccos(cos_alpha_a)

        # Base angle from motor A to end-effector position
        base_angle_a = np.arctan2(dy_a, dx_a)
        
        # Elbow-up/outer assembly configuration choice (+)
        theta_a = base_angle_a + alpha_a

        # --- Solve Motor B side ---
        dx_b = x - self.motor_b_x
        dy_b = y - self.motor_y
        R_b = np.hypot(dx_b, dy_b)

        # Check reachability triangle limits for Side B
        if R_b > (self.l2 + self.l4) or R_b < abs(self.l2 - self.l4) or R_b == 0:
            return None

        cos_alpha_b = (self.l2**2 + R_b**2 - self.l4**2) / (2.0 * self.l2 * R_b)
        cos_alpha_b = np.clip(cos_alpha_b, -1.0, 1.0)
        alpha_b = np.arccos(cos_alpha_b)

        base_angle_b = np.arctan2(dy_b, dx_b)
        
        # Elbow-up/outer assembly configuration choice (-)
        theta_b = base_angle_b - alpha_b

        # Map positions into interface contract bounds (-pi to +pi)
        theta_a = np.arctan2(np.sin(theta_a), np.cos(theta_a))
        theta_b = np.arctan2(np.sin(theta_b), np.cos(theta_b))

        return theta_a, theta_b

    def pose_callback(self, msg):
        target_x = msg.x
        target_y = msg.y
        
        angles = self.ik_5bar(target_x, target_y)
        
        if angles is None:
            self.get_logger().warn(f"Target position ({target_x}, {target_y}) out of reachable workspace!")
            return

        theta_a, theta_b = angles

        # Assemble the standard JointState payload matching Interface Contract
        msg_out = Float32MultiArray()
        msg_out.data = [float(theta_a), float(theta_b)]

        self.joint_pub.publish(msg_out)
        self.get_logger().info(f"Target ({target_x:.1f}, {target_y:.1f}) -> Sent Array: L={theta_a:.4f}, R={theta_b:.4f}")


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

