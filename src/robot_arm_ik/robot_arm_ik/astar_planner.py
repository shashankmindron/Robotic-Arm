import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetStateValidity
import numpy as np
import math

from five_bar_kinematics import FiveBarKinematics

class NodeObj:
    """Represents a specific state in our 3D Maze (X, Y, Layer)."""
    def __init__(self, x, y, branch, parent=None, action=None):
        self.x = x
        self.y = y
        self.branch = branch
        self.parent = parent
        self.action = action

        self.g = 0.0  
        self.h = 0.0  
        self.f = 0.0  

    def state(self):
        return (self.x, self.y, self.branch)
        
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.branch == other.branch

class AStarPlanner(Node):
    def __init__(self, step_size=10.0):
        super().__init__('astar_planner_node')
        self.step_size = step_size
        self.kinematics = FiveBarKinematics()
        
        # --- MoveIt Collision Client ---
        self.cli = self.create_client(GetStateValidity, '/check_state_validity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveIt Collision Service...')
            
        self.collision_cache = {} # The Speed-Up Memory!
        
        # Penalties
        self.WALK_PENALTY = step_size  
        self.FLIP_PENALTY = 500.0      

        # Portal Maps
        self.flip_A_map = {'out_out': 'in_out', 'in_out': 'out_out', 'out_in': 'in_in', 'in_in': 'out_in'}
        self.flip_B_map = {'out_out': 'out_in', 'out_in': 'out_out', 'in_out': 'in_in', 'in_in': 'in_out'}

    def check_collision(self, joint_angles):
        """Asks MoveIt if the 4 joint angles crash the aluminum arms together."""
        # Round to 3 decimal places to ensure cache hits
        cache_key = tuple(np.round(joint_angles, 3))
        
        if cache_key in self.collision_cache:
            return self.collision_cache[cache_key]

        req = GetStateValidity.Request()
        req.group_name = "arm_group"
        
        js = JointState()
        js.name = ['joint_a', 'joint_b', 'joint_distal_a', 'joint_distal_b']
        js.position = [float(joint_angles[0]), float(joint_angles[1]), 
                       float(joint_angles[2]), float(joint_angles[3])]
        req.robot_state.joint_state = js
        
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        is_safe = future.result().valid
        
        self.collision_cache[cache_key] = is_safe # Save to memory!
        return is_safe

    def get_valid_neighbors(self, current_node):
        neighbors = []
        cx, cy, c_branch = current_node.x, current_node.y, current_node.branch

        # --- 1. WALKING MOVES ---
        walk_moves = [
            (cx + self.step_size, cy), (cx - self.step_size, cy),
            (cx, cy + self.step_size), (cx, cy - self.step_size)
        ]

        for nx, ny in walk_moves:
            angles = self.kinematics.ik_5bar(nx, ny, filter_singularities=False)
            if angles is not None and c_branch in angles:
                theta_a, theta_b, _, _ = angles[c_branch]
                
                # Check 1: Red Zone (Math Singularity)
                if not self.kinematics.check_singularity(theta_a, theta_b, nx, ny):
                    # Check 2: Blue Zone (Physical MoveIt Collision)
                    if self.check_collision(angles[c_branch]):
                        walk_node = NodeObj(nx, ny, c_branch, parent=current_node, action='walk')
                        walk_node.g = current_node.g + self.WALK_PENALTY
                        neighbors.append(walk_node)

        # --- 2. PORTAL MOVES ---
        max_reach = self.kinematics.l1 + self.kinematics.l3
        min_reach = abs(self.kinematics.l1 - self.kinematics.l3)
        tol = self.step_size 
        
        R_a = np.hypot(cx - self.kinematics.motor_a_x, cy - self.kinematics.motor_y)
        R_b = np.hypot(cx - self.kinematics.motor_b_x, cy - self.kinematics.motor_y)
        
        a_can_flip = abs(max_reach - R_a) < tol or abs(R_a - min_reach) < tol
        b_can_flip = abs(max_reach - R_b) < tol or abs(R_b - min_reach) < tol

        # Only allow the flip if the DESTINATION layer isn't inside a physical collision!
        current_angles = self.kinematics.ik_5bar(cx, cy, filter_singularities=False)

        if a_can_flip:
            new_branch = self.flip_A_map[c_branch]
            if new_branch in current_angles and self.check_collision(current_angles[new_branch]):
                flip_node = NodeObj(cx, cy, new_branch, parent=current_node, action='flip_a')
                flip_node.g = current_node.g + self.FLIP_PENALTY
                neighbors.append(flip_node)
            
        if b_can_flip:
            new_branch = self.flip_B_map[c_branch]
            if new_branch in current_angles and self.check_collision(current_angles[new_branch]):
                flip_node = NodeObj(cx, cy, new_branch, parent=current_node, action='flip_b')
                flip_node.g = current_node.g + self.FLIP_PENALTY
                neighbors.append(flip_node)

        return neighbors

    def heuristic(self, node, target_x, target_y):
        dx = abs(node.x - target_x)
        dy = abs(node.y - target_y)
        return (dx + dy) * (self.WALK_PENALTY / self.step_size)

    def reconstruct_path(self, finish_node):
        self.get_logger().info("Target reached! Compiling trajectory array...")
        trajectory = []
        current = finish_node
        
        while current is not None:
            if current.parent is None: break 

            if current.action == 'walk':
                step = {"action": "ik_move", "target": {"x": float(current.x), "y": float(current.y)}, "locked_branch": current.branch}
            elif current.action == 'flip_a':
                step = {"action": "fk_portal_flip", "motor_to_pulse": "Motor_A", "new_branch": current.branch}
            elif current.action == 'flip_b':
                step = {"action": "fk_portal_flip", "motor_to_pulse": "Motor_B", "new_branch": current.branch}
                
            trajectory.append(step)
            current = current.parent
            
        trajectory.reverse()
        return trajectory

    def plan_trajectory(self, start_x, start_y, start_branch, target_x, target_y):
        self.get_logger().info(f"Goal: {target_x}, {target_y} | Starting from: {start_x}, {start_y} in {start_branch}")

        target_solutions = self.kinematics.ik_5bar(target_x, target_y, filter_singularities=True)
        if not target_solutions:
            self.get_logger().error("Target is completely out of bounds or inside a permanent deadzone.")
            return None
            
        # Verify target isn't inside a physical collision
        target_safe = False
        for branch, angles in target_solutions.items():
            if self.check_collision(angles):
                target_safe = True
                break
                
        if not target_safe:
            self.get_logger().error("Target causes a physical MoveIt collision.")
            return None

        start_node = NodeObj(start_x, start_y, start_branch)
        start_node.h = self.heuristic(start_node, target_x, target_y)
        start_node.f = start_node.g + start_node.h

        open_set = [start_node]
        closed_set = set() 

        while len(open_set) > 0:
            open_set.sort(key=lambda n: n.f)
            current = open_set.pop(0)

            if abs(current.x - target_x) < 1.0 and abs(current.y - target_y) < 1.0:
                return self.reconstruct_path(current)

            closed_set.add(current.state())

            for neighbor in self.get_valid_neighbors(current):
                if neighbor.state() in closed_set:
                    continue

                neighbor.h = self.heuristic(neighbor, target_x, target_y)
                neighbor.f = neighbor.g + neighbor.h

                existing = next((n for n in open_set if n.state() == neighbor.state()), None)
                if existing:
                    if neighbor.g >= existing.g: continue 
                    else: open_set.remove(existing) 

                open_set.append(neighbor)

        self.get_logger().error("Target is trapped behind a wall. No valid path exists.")
        return None