import numpy as np
import math

class TrajectoryPlanner:
    def __init__(self, kinematics_engine, collision_guard):
        self.kin = kinematics_engine
        self.guard = collision_guard
        
    def generate_straight_line(self, start_x, start_y, end_x, end_y, step_mm=1.0):
        """
        Step 5: The Linear Interpolator
        Breaks a straight line into a breadcrumb trail of 1mm waypoints.
        Returns a list of (X, Y) tuples.
        """
        distance = np.hypot(end_x - start_x, end_y - start_y)
        num_steps = max(int(distance / step_mm), 2)
        
        x_waypoints = np.linspace(start_x, end_x, num_steps)
        y_waypoints = np.linspace(start_y, end_y, num_steps)
        
        return list(zip(x_waypoints, y_waypoints))

    def generate_velocity_profile(self, total_distance, step_mm, v_max, accel, decel):
        """
        Step 6: The Trapezoidal Velocity Profile
        Assigns a target Cartesian velocity (mm/s) to each waypoint.
        """
        num_steps = max(int(total_distance / step_mm), 2)
        velocities = []
        
        for i in range(num_steps):
            # Current distance traveled along the line
            s = i * step_mm
            # Distance remaining to the end of the line
            s_remain = total_distance - s
            
            # Kinematics Equation: v = sqrt(2 * a * d)
            # 1. How fast can we go based on our acceleration from 0?
            v_accel = math.sqrt(2 * accel * s) if s > 0 else 0.0
            
            # 2. How fast can we go so we can still brake in time?
            v_decel = math.sqrt(2 * decel * s_remain) if s_remain > 0 else 0.0
            
            # 3. The actual speed is the lowest of the three constraints
            # (Accelerating, Braking, or Cruising at Max Speed)
            v_target = min(v_accel, v_decel, v_max)
            
            # Prevent the speed from ever being exactly 0 (except at the very end)
            # otherwise the robot would stall mid-move
            v_target = max(v_target, 1.0) 
            
            velocities.append(v_target)
            
        return velocities

    def select_best_branch(self, current_angles, target_x, target_y):
        """
        Step 4: The Cost Function (Branch Selector)
        Evaluates all 4 IK solutions for a target.
        Discards unsafe (collision) solutions using MoveIt.
        Picks the safe solution that requires the smallest angular movement.
        """
        solutions = self.kin.ik_5bar(target_x, target_y)
        
        if not solutions:
            return None, "Mathematically unreachable"
            
        best_branch = None
        min_cost = float('inf')
        best_angles = None
        
        def shortest_angular_dist(a1, a2):
            diff = (a1 - a2 + np.pi) % (2 * np.pi) - np.pi
            return abs(diff)

        for name, angles in solutions.items():
            if not self.guard.check_collision(angles):
                continue
                
            delta_a = shortest_angular_dist(angles[0], current_angles[0])
            delta_b = shortest_angular_dist(angles[1], current_angles[1])
            cost = delta_a + delta_b
            
            if cost < min_cost:
                min_cost = cost
                best_branch = name
                best_angles = angles
                
        if best_angles is None:
            return None, "All math solutions cause physical STLs to crash"
            
        return best_angles, best_branch