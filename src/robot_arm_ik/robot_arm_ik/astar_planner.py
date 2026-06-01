import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetStateValidity
import numpy as np
import math

from robot_arm_ik.five_bar_kinematics import FiveBarKinematics

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
    def __init__(self, step_size=15.0): # Adjusted to your Goldilocks value!
        super().__init__('astar_planner_node')
        self.step_size = step_size
        self.kinematics = FiveBarKinematics()
        
        # --- MoveIt Collision Client ---
        self.cli = self.create_client(GetStateValidity, '/check_state_validity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveIt Collision Service...')
            
        self.collision_cache = {} 
        
        # --- PENALTIES ---
        self.FLIP_PENALTY = 5000.0      
        self.DANGER_MULTIPLIER = 50.0  

        self.flip_A_map = {'out_out': 'in_out', 'in_out': 'out_out', 'out_in': 'in_in', 'in_in': 'out_in'}
        self.flip_B_map = {'out_out': 'out_in', 'out_in': 'out_out', 'in_out': 'in_in', 'in_in': 'in_out'}

    def check_collision(self, joint_angles, x_coord):
        """
        BROAD-PHASE OPTIMIZATION: 
        If the end-effector is physically on the safe side of the board (X <= 175),
        bypass MoveIt completely to save massive amounts of CPU time!
        """
        if x_coord <= 175.0:
            return True

        # If X > 175, check our local memory cache
        cache_key = tuple(np.round(joint_angles, 3))
        if cache_key in self.collision_cache:
            return self.collision_cache[cache_key]

        # If it's not in memory, ask MoveIt
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
        
        self.collision_cache[cache_key] = is_safe 
        return is_safe

    def get_valid_neighbors(self, current_node):
        neighbors = []
        cx, cy, c_branch = current_node.x, current_node.y, current_node.branch

        # --- 8-WAY MOVEMENT ---
        diag = self.step_size * 1.4142 
        
        walk_moves = [
            (cx + self.step_size, cy, self.step_size), 
            (cx - self.step_size, cy, self.step_size), 
            (cx, cy + self.step_size, self.step_size), 
            (cx, cy - self.step_size, self.step_size), 
            (cx + self.step_size, cy + self.step_size, diag), 
            (cx - self.step_size, cy + self.step_size, diag), 
            (cx + self.step_size, cy - self.step_size, diag), 
            (cx - self.step_size, cy - self.step_size, diag)  
        ]

        for nx, ny, step_cost in walk_moves:
            angles = self.kinematics.ik_5bar(nx, ny, filter_singularities=False)
            if angles is not None and c_branch in angles:
                theta_a = angles[c_branch][0]
                theta_b = angles[c_branch][1]
                theta_da = angles[c_branch][2] 
                theta_db = angles[c_branch][3] 
                
                if not self.kinematics.check_singularity(theta_a, theta_b, nx, ny):
                    
                    # Pass 'nx' into the optimized collision checker
                    if self.check_collision(angles[c_branch], nx):
                        
                        transmission_quality = abs(math.sin(theta_da - theta_db))
                        danger_score = 1.0 - transmission_quality
                        
                        dynamic_walk_cost = step_cost + ((danger_score ** 2) * self.DANGER_MULTIPLIER)
                        
                        walk_node = NodeObj(nx, ny, c_branch, parent=current_node, action='walk')
                        walk_node.g = current_node.g + dynamic_walk_cost
                        neighbors.append(walk_node)

        # --- PORTAL MOVES ---
        max_reach = self.kinematics.l1 + self.kinematics.l3
        min_reach = abs(self.kinematics.l1 - self.kinematics.l3)
        tol = self.step_size 
        
        R_a = np.hypot(cx - self.kinematics.motor_a_x, cy - self.kinematics.motor_y)
        R_b = np.hypot(cx - self.kinematics.motor_b_x, cy - self.kinematics.motor_y)
        
        a_can_flip = abs(max_reach - R_a) < tol or abs(R_a - min_reach) < tol
        b_can_flip = abs(max_reach - R_b) < tol or abs(R_b - min_reach) < tol

        current_angles = self.kinematics.ik_5bar(cx, cy, filter_singularities=False)

        if a_can_flip:
            new_branch = self.flip_A_map[c_branch]
            # Pass 'cx' into the optimized collision checker
            if new_branch in current_angles and self.check_collision(current_angles[new_branch], cx):
                flip_node = NodeObj(cx, cy, new_branch, parent=current_node, action='flip_a')
                flip_node.g = current_node.g + self.FLIP_PENALTY
                neighbors.append(flip_node)
            
        if b_can_flip:
            new_branch = self.flip_B_map[c_branch]
            # Pass 'cx' into the optimized collision checker
            if new_branch in current_angles and self.check_collision(current_angles[new_branch], cx):
                flip_node = NodeObj(cx, cy, new_branch, parent=current_node, action='flip_b')
                flip_node.g = current_node.g + self.FLIP_PENALTY
                neighbors.append(flip_node)

        return neighbors

    def heuristic(self, node, target_x, target_y):
        # Weighted A* for speed
        base_distance = math.hypot(node.x - target_x, node.y - target_y)
        return base_distance * 2.5 

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

    # ==============================================================================
    # THE STRING PULLER (Trajectory Smoothing)
    # ==============================================================================
    def check_line_of_sight(self, node_a, node_b):
        """
        Draws a virtual straight line between two points and samples it every 5mm.
        Returns True if the entire line is safe from collisions and singularities.
        """
        x1, y1 = node_a['target']['x'], node_a['target']['y']
        x2, y2 = node_b['target']['x'], node_b['target']['y']
        branch = node_a['locked_branch']
        
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist == 0: return True
        
        # Sample the line every 5mm
        steps = max(2, int(dist / 5.0)) 
        
        for i in range(1, steps):
            t = i / steps
            cx = x1 + t * (x2 - x1)
            cy = y1 + t * (y2 - y1)
            
            angles = self.kinematics.ik_5bar(cx, cy, filter_singularities=False)
            if not angles or branch not in angles: 
                return False
                
            theta_a, theta_b, theta_da, theta_db = angles[branch]
            
            # 1. Math Singularity Check
            if self.kinematics.check_singularity(theta_a, theta_b, cx, cy): 
                return False
                
            # 2. Hard Buffer Check (Keep the straight line away from the red zone edge)
            transmission_quality = abs(math.sin(theta_da - theta_db))
            if transmission_quality < 0.15: 
                return False 
                
            # 3. Physical Collision Check
            if not self.check_collision(angles[branch], cx): 
                return False
                
        return True

    def smooth_trajectory(self, raw_trajectory):
        """
        The 'String Puller' algorithm. Skips intermediate waypoints if a direct
        line of sight exists between an earlier node and a later node.
        """
        self.get_logger().info("Pulling the string to smooth the zig-zags...")
        if len(raw_trajectory) <= 2:
            return raw_trajectory

        smoothed_path = []
        i = 0
        
        while i < len(raw_trajectory):
            smoothed_path.append(raw_trajectory[i])
            
            # We CANNOT smooth across a portal flip! 
            # If the current step is a flip, just move to the next frame.
            if raw_trajectory[i]['action'] == 'fk_portal_flip':
                i += 1
                continue
                
            furthest_visible = i + 1
            
            # Look ahead to find the absolute furthest point we can see in a straight line
            for j in range(i + 2, len(raw_trajectory)):
                
                # Stop looking ahead if we hit a portal boundary
                if raw_trajectory[j]['action'] == 'fk_portal_flip':
                    break 
                    
                if self.check_line_of_sight(raw_trajectory[i], raw_trajectory[j]):
                    furthest_visible = j
                else:
                    # If we can't see 'j', we definitely can't see 'j+1'. Stop looking.
                    break 
                    
            # Skip all the jagged nodes in between and jump straight to the furthest visible point!
            i = furthest_visible
            
        return smoothed_path

    # ==============================================================================
    # CORE ROUTING ALGORITHM
    # ==============================================================================
    def plan_trajectory(self, start_x, start_y, start_branch, target_x, target_y):
        self.get_logger().info(f"Goal: {target_x}, {target_y} | Starting from: {start_x}, {start_y} in {start_branch}")

        target_solutions = self.kinematics.ik_5bar(target_x, target_y, filter_singularities=True)
        if not target_solutions:
            self.get_logger().error("Target is completely out of bounds or inside a permanent deadzone.")
            return None
            
        target_safe = False
        for branch, angles in target_solutions.items():
            # Pass target_x into the optimized collision checker
            if self.check_collision(angles, target_x):
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

            if math.hypot(current.x - target_x, current.y - target_y) <= (self.step_size * 1.5):
                
                # Victory Condition Hit -> Generate Path -> Pull String -> Return Optimized Path
                if abs(current.x - target_x) > 0.1 or abs(current.y - target_y) > 0.1:
                    final_node = NodeObj(target_x, target_y, current.branch, parent=current, action='walk')
                    raw_path = self.reconstruct_path(final_node)
                    return self.smooth_trajectory(raw_path)
                    
                raw_path = self.reconstruct_path(current)
                return self.smooth_trajectory(raw_path)

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