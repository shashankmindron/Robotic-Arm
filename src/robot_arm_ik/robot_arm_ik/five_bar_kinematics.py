import numpy as np

class FiveBarKinematics:
    def __init__(self):
        # ----------------------------------------------------------------------
        # HARDWARE CONFIGURATION
        # ----------------------------------------------------------------------
        self.d  = 350.0   
        self.l1 = 200.0   
        self.l2 = 200.0   
        self.l3 = 290.0   
        self.l4 = 290.0   

        self.motor_a_x = -self.d / 2.0   
        self.motor_b_x =  self.d / 2.0   
        self.motor_y   = 0.0
        
        # State Tracker (Initial guess: pointing straight up)
        self.last_known_xy = (0.0, 400.0) 

    def fk_5bar(self, theta_a, theta_b):
        """Forward Kinematics: Angles -> (X, Y)"""
        elbow_a_x = self.motor_a_x + self.l1 * np.cos(theta_a)
        elbow_a_y = self.motor_y   + self.l1 * np.sin(theta_a)
        
        elbow_b_x = self.motor_b_x + self.l2 * np.cos(theta_b)
        elbow_b_y = self.motor_y   + self.l2 * np.sin(theta_b)

        dx = elbow_b_x - elbow_a_x
        dy = elbow_b_y - elbow_a_y
        dist = np.hypot(dx, dy)

        if dist > (self.l3 + self.l4) or dist < abs(self.l3 - self.l4) or dist == 0:
            return None

        a = (self.l3**2 - self.l4**2 + dist**2) / (2 * dist)
        h = np.sqrt(abs(self.l3**2 - a**2))

        p2_x = elbow_a_x + a * (dx / dist)
        p2_y = elbow_a_y + a * (dy / dist)

        x1 = p2_x + h * (dy / dist)
        y1 = p2_y - h * (dx / dist)
        
        x2 = p2_x - h * (dy / dist)
        y2 = p2_y + h * (dx / dist)

        dist_to_1 = np.hypot(x1 - self.last_known_xy[0], y1 - self.last_known_xy[1])
        dist_to_2 = np.hypot(x2 - self.last_known_xy[0], y2 - self.last_known_xy[1])

        correct_xy = (x1, y1) if dist_to_1 < dist_to_2 else (x2, y2)
        self.last_known_xy = correct_xy
        
        return correct_xy

    def ik_5bar(self, x, y):
        """
        Inverse Kinematics: (X, Y) -> 4 Possible configurations 
        Returns a dictionary containing tuples of 4 angles:
        (theta_a, theta_b, theta_distal_a, theta_distal_b) in radians.
        """
        valid_solutions = {}
        norm = lambda ang: np.arctan2(np.sin(ang), np.cos(ang))

        # --- Motor A (Left) ---
        dx_a = x - self.motor_a_x
        dy_a = y - self.motor_y
        R_a  = np.hypot(dx_a, dy_a)

        if R_a > (self.l1 + self.l3) or R_a < abs(self.l1 - self.l3):
            return None 

        cos_alpha_a = (self.l1**2 + R_a**2 - self.l3**2) / (2.0 * self.l1 * R_a)
        alpha_a = np.arccos(np.clip(cos_alpha_a, -1.0, 1.0))
        base_angle_a = np.arctan2(dy_a, dx_a)

        theta_a_out = base_angle_a + alpha_a
        theta_a_in  = base_angle_a - alpha_a

        # --- Motor B (Right) ---
        dx_b = x - self.motor_b_x
        dy_b = y - self.motor_y
        R_b  = np.hypot(dx_b, dy_b)

        if R_b > (self.l2 + self.l4) or R_b < abs(self.l2 - self.l4):
            return None 

        cos_alpha_b = (self.l2**2 + R_b**2 - self.l4**2) / (2.0 * self.l2 * R_b)
        alpha_b = np.arccos(np.clip(cos_alpha_b, -1.0, 1.0))
        base_angle_b = np.arctan2(dy_b, dx_b)

        theta_b_out = base_angle_b - alpha_b
        theta_b_in  = base_angle_b + alpha_b

        # --- Helper: Calculate relative distal angles ---
        def calc_distal(theta_prox, elbow_x, elbow_y, target_x, target_y):
            # Absolute angle of the vector from elbow to target
            abs_distal_angle = np.arctan2(target_y - elbow_y, target_x - elbow_x)
            # URDF requires the relative angle between distal and proximal link
            return norm(abs_distal_angle - theta_prox)

        # Build permutations and calculate distal angles for each
        permutations = [
            ("out_out", theta_a_out, theta_b_out),
            ("in_in",   theta_a_in,  theta_b_in),
            ("out_in",  theta_a_out, theta_b_in),
            ("in_out",  theta_a_in,  theta_b_out)
        ]

        for name, th_a, th_b in permutations:
            # 1. Find where the elbows are in this specific configuration
            elbow_a_x = self.motor_a_x + self.l1 * np.cos(th_a)
            elbow_a_y = self.motor_y   + self.l1 * np.sin(th_a)
            
            elbow_b_x = self.motor_b_x + self.l2 * np.cos(th_b)
            elbow_b_y = self.motor_y   + self.l2 * np.sin(th_b)

            # 2. Calculate the relative joint angle to reach the target (X,Y)
            distal_a = calc_distal(th_a, elbow_a_x, elbow_a_y, x, y)
            distal_b = calc_distal(th_b, elbow_b_x, elbow_b_y, x, y)

            # 3. Store all 4 joints!
            valid_solutions[name] = (norm(th_a), norm(th_b), distal_a, distal_b)

        return valid_solutions


# --- The Validation Test ---
if __name__ == '__main__':
    kin = FiveBarKinematics()
    
    print("--- TEST 1: Forward Kinematics ---")
    start_angle = np.pi / 2.0  # 90 degrees
    pos = kin.fk_5bar(start_angle, start_angle)
    print(f"Input:  Motors at 90 deg")
    print(f"Output: X={pos[0]:.2f}, Y={pos[1]:.2f}")
    
    print("\n--- TEST 2: Inverse Kinematics ---")
    print(f"Input:  Targeting the exact position we just found (X={pos[0]:.2f}, Y={pos[1]:.2f})")
    solutions = kin.ik_5bar(pos[0], pos[1])
    
    if solutions:
        for config_name, angles in solutions.items():
            deg_a = np.degrees(angles[0])
            deg_b = np.degrees(angles[1])
            print(f"Output [{config_name}]: Motor A = {deg_a:6.2f} deg, Motor B = {deg_b:6.2f} deg")
    else:
        print("Target out of reach!")