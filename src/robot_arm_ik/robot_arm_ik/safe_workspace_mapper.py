import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetStateValidity
import numpy as np
import matplotlib.pyplot as plt

# Import your custom math brain!
from five_bar_kinematics import FiveBarKinematics

class SafeWorkspaceMapper(Node):
    def __init__(self):
        super().__init__('safe_workspace_mapper')
        
        # Create a client to talk to MoveIt's collision engine
        self.cli = self.create_client(GetStateValidity, '/check_state_validity')
        
        # Wait for MoveIt to boot up
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveIt Collision Service...')
            
        self.kinematics = FiveBarKinematics()

    def check_collision(self, joint_angles):
        """Sends 4 joint angles to MoveIt and returns True if safe, False if crash."""
        req = GetStateValidity.Request()
        req.group_name = "arm_group" # Ensure this matches your SRDF group name!
        
        # Package the 4 angles into a standard ROS JointState message
        js = JointState()
        js.name = ['joint_a', 'joint_b', 'joint_distal_a', 'joint_distal_b']
        js.position = [float(joint_angles[0]), float(joint_angles[1]), 
                       float(joint_angles[2]), float(joint_angles[3])]
        
        req.robot_state.joint_state = js
        
        # Ask MoveIt! 
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result().valid

    def map_workspace(self):
        self.get_logger().info("Starting Physical Workspace Scan for ALL branches...")
        
        # Grid parameters (Using a 10mm step. Smaller steps take drastically longer!)
        x_vals = np.arange(-500, 500, 10)
        y_vals = np.arange(-700, 700, 10)
        
        # Initialize dictionaries to hold the coordinate data for all 4 configurations
        branches = ["out_out", "in_in", "out_in", "in_out"]
        
        safe_pts = {b: {'x': [], 'y': []} for b in branches}
        collision_pts = {b: {'x': [], 'y': []} for b in branches}
        
        total_points = len(x_vals) * len(y_vals)
        count = 0

        for x in x_vals:
            for y in y_vals:
                count += 1
                if count % 200 == 0:
                    self.get_logger().info(f"Scanned {count} / {total_points} coordinates...")

                # 1. Ask the Math: Can we reach this?
                solutions = self.kinematics.ik_5bar(x, y)
                
                if solutions is not None:
                    # 2. Iterate through ALL 4 mathematical configurations for this pixel
                    for branch_name, angles in solutions.items():
                        
                        # 3. Ask MoveIt: Does this specific configuration crash the STLs?
                        is_safe = self.check_collision(angles)
                        
                        if is_safe:
                            safe_pts[branch_name]['x'].append(x)
                            safe_pts[branch_name]['y'].append(y)
                        else:
                            collision_pts[branch_name]['x'].append(x)
                            collision_pts[branch_name]['y'].append(y)

        self.get_logger().info("Scan Complete! Plotting results...")
        self.plot_results(safe_pts, collision_pts, branches)

    def plot_results(self, safe_pts, collision_pts, branches):
        # Create a 2x2 grid of subplots for the 4 branches
        fig, axs = plt.subplots(2, 2, figsize=(14, 12))
        fig.suptitle('True Physical Workspace by Configuration', fontsize=16)
        
        # Flatten the 2x2 matrix of axes so we can loop over them easily
        axs = axs.flatten()

        for idx, branch_name in enumerate(branches):
            ax = axs[idx]
            
            # Plot Collisions (Red)
            ax.scatter(collision_pts[branch_name]['x'], collision_pts[branch_name]['y'], 
                       c='red', s=10, alpha=0.5, label='Collision')
            
            # Plot Safe Zones (Green)
            ax.scatter(safe_pts[branch_name]['x'], safe_pts[branch_name]['y'], 
                       c='green', s=10, alpha=0.7, label='Safe')
            
            # Plot Motors for reference
            ax.plot(self.kinematics.motor_a_x, self.kinematics.motor_y, 'ko', markersize=8)
            ax.plot(self.kinematics.motor_b_x, self.kinematics.motor_y, 'ko', markersize=8)
            
            ax.set_title(f'Configuration: {branch_name}')
            ax.set_xlabel('X (mm)')
            ax.set_ylabel('Y (mm)')
            ax.axis('equal')
            ax.grid(True)
            if idx == 0:
                ax.legend(loc='upper right')

        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    mapper = SafeWorkspaceMapper()
    mapper.map_workspace()
    mapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()