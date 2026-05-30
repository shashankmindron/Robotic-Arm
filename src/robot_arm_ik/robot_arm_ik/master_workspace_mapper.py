import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetStateValidity
import numpy as np
import matplotlib.pyplot as plt

# Using your custom math brain properly!
from five_bar_kinematics import FiveBarKinematics
from astar_planner import AStarPlanner

class MasterWorkspaceMapper(Node):
    def __init__(self):
        super().__init__('master_workspace_mapper')
        
        self.cli = self.create_client(GetStateValidity, '/check_state_validity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveIt Collision Service...')
            
        self.kinematics = FiveBarKinematics()

    def check_collision(self, joint_angles):
        req = GetStateValidity.Request()
        req.group_name = "arm_group" 
        
        js = JointState()
        js.name = ['joint_a', 'joint_b', 'joint_distal_a', 'joint_distal_b']
        js.position = [float(joint_angles[0]), float(joint_angles[1]), 
                       float(joint_angles[2]), float(joint_angles[3])]
        req.robot_state.joint_state = js
        
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().valid

    def check_type1(self, x, y):
        """
        Calculates if the (X,Y) coordinate is near the absolute physical limit (Type 1 Singularity)
        for Arm A, Arm B, or Both. This identifies the 'Flip Zones'.
        """
        max_reach = self.kinematics.l1 + self.kinematics.l3 # 490 mm
        min_reach = abs(self.kinematics.l1 - self.kinematics.l3) # 90 mm
        
        R_a = np.hypot(x - self.kinematics.motor_a_x, y - self.kinematics.motor_y)
        R_b = np.hypot(x - self.kinematics.motor_b_x, y - self.kinematics.motor_y)
        
        tol = 5.0 
        
        a_is_type1 = abs(max_reach - R_a) < tol or abs(R_a - min_reach) < tol
        b_is_type1 = abs(max_reach - R_b) < tol or abs(R_b - min_reach) < tol
        
        if a_is_type1 and b_is_type1:
            return 'Both'
        elif a_is_type1:
            return 'A'
        elif b_is_type1:
            return 'B'
        
        return None

    def map_workspace(self, trajectory=None):
        self.get_logger().info("Starting Deep Scan: Collisions, Deadzones, & Flip Zones...")
        
        x_vals = np.arange(-350, 350, 10) # Using 10mm step to generate maps faster
        y_vals = np.arange(-500, 500, 10)
        
        branches = ["out_out", "in_out", "out_in", "in_in"]
        
        safe_pts = {b: {'x': [], 'y': []} for b in branches}
        collision_pts = {b: {'x': [], 'y': []} for b in branches}
        singularity_pts = {b: {'x': [], 'y': []} for b in branches}
        
        type1_a_pts = {b: {'x': [], 'y': []} for b in branches}
        type1_b_pts = {b: {'x': [], 'y': []} for b in branches}
        type1_both_pts = {b: {'x': [], 'y': []} for b in branches}
        
        total_points = len(x_vals) * len(y_vals)
        count = 0

        for x in x_vals:
            for y in y_vals:
                count += 1
                if count % 200 == 0:
                    self.get_logger().info(f"Scanned {count} / {total_points} coordinates...")

                solutions = self.kinematics.ik_5bar(x, y, filter_singularities=False)
                
                if solutions is not None:
                    type1_status = self.check_type1(x, y)

                    for branch_name, angles in solutions.items():
                        
                        if self.kinematics.check_singularity(angles[0], angles[1], x, y):
                            singularity_pts[branch_name]['x'].append(x)
                            singularity_pts[branch_name]['y'].append(y)
                            continue 
                        
                        if not self.check_collision(angles):
                            collision_pts[branch_name]['x'].append(x)
                            collision_pts[branch_name]['y'].append(y)
                            continue 
                            
                        if type1_status == 'Both':
                            type1_both_pts[branch_name]['x'].append(x)
                            type1_both_pts[branch_name]['y'].append(y)
                        elif type1_status == 'A':
                            type1_a_pts[branch_name]['x'].append(x)
                            type1_a_pts[branch_name]['y'].append(y)
                        elif type1_status == 'B':
                            type1_b_pts[branch_name]['x'].append(x)
                            type1_b_pts[branch_name]['y'].append(y)
                        else:
                            safe_pts[branch_name]['x'].append(x)
                            safe_pts[branch_name]['y'].append(y)

        self.get_logger().info("Scan Complete! Rendering plots...")
        self.plot_results(safe_pts, collision_pts, singularity_pts, type1_a_pts, type1_b_pts, type1_both_pts, branches, trajectory)

    def plot_results(self, safe_pts, collision_pts, singularity_pts, type1_a_pts, type1_b_pts, type1_both_pts, branches, trajectory=None):
        fig, axs = plt.subplots(2, 2, figsize=(16, 14))
        fig.suptitle('A* Pathfinding Overlay: The Portal Route', fontsize=16)
        axs = axs.flatten()

        for idx, branch_name in enumerate(branches):
            ax = axs[idx]
            
            ax.scatter(collision_pts[branch_name]['x'], collision_pts[branch_name]['y'], c='blue', s=12, alpha=0.1, label='Collision')
            ax.scatter(safe_pts[branch_name]['x'], safe_pts[branch_name]['y'], c='green', s=12, alpha=0.2, label='Safe')
            ax.scatter(singularity_pts[branch_name]['x'], singularity_pts[branch_name]['y'], c='red', s=15, alpha=0.6, label='Deadzone')
            
            ax.scatter(type1_a_pts[branch_name]['x'], type1_a_pts[branch_name]['y'], c='cyan', s=25, alpha=0.8, label='Portal A')
            ax.scatter(type1_b_pts[branch_name]['x'], type1_b_pts[branch_name]['y'], c='magenta', s=25, alpha=0.8, label='Portal B')
            ax.scatter(type1_both_pts[branch_name]['x'], type1_both_pts[branch_name]['y'], c='orange', s=35, alpha=1.0, label='Portal Both')
            
            ax.plot(self.kinematics.motor_a_x, self.kinematics.motor_y, 'ko', markersize=10)
            ax.plot(self.kinematics.motor_b_x, self.kinematics.motor_y, 'ko', markersize=10)
            
            ax.set_title(f'Branch: {branch_name}', fontweight='bold')
            ax.set_xlabel('X (mm)')
            ax.set_ylabel('Y (mm)')
            ax.set_xlim([-400, 400])
            ax.set_ylim([-550, 550])
            ax.axis('equal')
            ax.grid(True, linestyle='--', alpha=0.6)
            
            if idx == 0: 
                ax.legend(loc='upper right', fontsize='small')

        # --- DRAW THE A* PATH OVERLAY ---
        if trajectory:
            segments = []
            current_seg = {'branch': None, 'x': [], 'y': []}

            for step in trajectory:
                if step['action'] == 'ik_move':
                    b = step['locked_branch']
                    if b != current_seg['branch']:
                        if current_seg['branch'] is not None:
                            segments.append(current_seg)
                        current_seg = {'branch': b, 'x': [], 'y': []}
                    current_seg['x'].append(step['target']['x'])
                    current_seg['y'].append(step['target']['y'])

            if current_seg['branch'] is not None:
                segments.append(current_seg)

            for seg in segments:
                branch_name = seg['branch']
                if branch_name in branches:
                    ax = axs[branches.index(branch_name)]
                    px = seg['x']
                    py = seg['y']

                    # White outline, black core
                    ax.plot(px, py, color='white', linewidth=6, zorder=9)
                    ax.plot(px, py, color='black', linewidth=3, zorder=10)
                    ax.scatter(px, py, color='black', s=10, zorder=11)

                    # Entry and Exit markers
                    ax.plot(px[0], py[0], marker='o', color='yellow', markersize=8, markeredgecolor='black', zorder=12)
                    ax.plot(px[-1], py[-1], marker='*', color='yellow', markersize=14, markeredgecolor='black', zorder=12)

        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    
    print("\n=======================================================")
    print("PHASE 1: Solving the Maze (A* Pathfinding)")
    print("=======================================================")
    
    planner = AStarPlanner(step_size=10.0)
    
    start_point = (0.0, 300.0)
    target_point = (0.0, -300.0)
    starting_layer = 'out_out'
    
    trajectory = planner.plan_trajectory(
        start_x=start_point[0], start_y=start_point[1], 
        start_branch=starting_layer, 
        target_x=target_point[0], target_y=target_point[1]
    )
    
    # Destroy the planner to free up the ROS 2 node name
    planner.destroy_node()

    print("\n=======================================================")
    print("PHASE 2: Generating Visualization Map")
    print("=======================================================")
    
    if trajectory:
        mapper = MasterWorkspaceMapper()
        mapper.map_workspace(trajectory=trajectory)
        mapper.destroy_node()
    else:
        print("Pathfinding failed. Cannot map trajectory.")
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()