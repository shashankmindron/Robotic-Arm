import numpy as np
import math
from five_bar_kinematics import FiveBarKinematics

def check_singularities():
    print("--- 5-BAR LINKAGE SINGULARITY STRESS TEST ---")
    kin = FiveBarKinematics()
    
    # Extract link lengths from your class to calculate exact limits
    l1 = kin.l1 # 200 mm
    l3 = kin.l3 # 290 mm
    max_reach = l1 + l3 # 490 mm from a motor pivot
    
    print("\n[TEST 1]: TYPE I SINGULARITY (Workspace Boundary)")
    print("Attempting to stretch Motor B's arm to its absolute maximum limit...")
    
    # Motor B is at X: 175, Y: 0. 
    # If we command X: 175, Y: 490, the arm must point straight up in a perfect line.
    # If we command X: 175, Y: 491, it is physically impossible.
    
    target_x = 175.0
    target_y_exact = max_reach
    target_y_impossible = max_reach + 1.0

    print(f" -> Testing Exact Boundary (X:{target_x}, Y:{target_y_exact})")
    solutions_exact = kin.ik_5bar(target_x, target_y_exact)
    if solutions_exact:
        print("    Result: SOLVED. (Math held together at the exact boundary)")
    else:
        print("    Result: FAILED. (Solver rejected the boundary)")

    print(f" -> Testing Impossible Point (X:{target_x}, Y:{target_y_impossible})")
    solutions_impossible = kin.ik_5bar(target_x, target_y_impossible)
    if not solutions_impossible:
        print("    Result: REJECTED cleanly! (Your Type I protection is working perfectly)")
    else:
        print("    Result: SOLVED?! (WARNING: Math returned a solution for an impossible point!)")


    print("\n[TEST 2]: TYPE II SINGULARITY (Interior/Parallel Deadzones)")
    print("Evaluating the internal angles of the distal arms (L3 and L4)...")
    
    # To test Type II, let's scan a grid inside the safe workspace 
    # and explicitly check the angle between the two distal arms.
    
    warning_triggered = False
    
    # Scanning a 100x100mm area in front of the robot
    for x in range(-175, 175, 10):
        for y in range(0, 450, 10):
            solutions = kin.ik_5bar(x, y)
            
            if solutions:
                # Check all kinematic branches returned
                for branch_name, angles in solutions.items():
                    # angles = (theta_a, theta_b, distal_a, distal_b)
                    distal_a_angle = angles[2]
                    distal_b_angle = angles[3]
                    
                    # Calculate the difference between the distal arm angles
                    angle_diff = abs(distal_a_angle - distal_b_angle)
                    
                    # Normalize the difference to check for 0 or 180 degrees (PI radians)
                    angle_diff = angle_diff % math.pi
                    
                    # If the difference is very close to 0 or PI, the arms are forming a straight line
                    if angle_diff < 0.05 or abs(angle_diff - math.pi) < 0.05:
                        print(f"    [!] TYPE II DANGER at (X:{x}, Y:{y}) on branch '{branch_name}'")
                        print(f"        Distal A: {math.degrees(distal_a_angle):.1f} deg | Distal B: {math.degrees(distal_b_angle):.1f} deg")
                        warning_triggered = True

    if not warning_triggered:
        print("    Result: No Type II singularities found in the tested grid.")
        print("    (Note: This just means this specific grid is safe. We need to actively guard against this during trajectory planning!)")
    
    print("\n--- TEST COMPLETE ---")

if __name__ == "__main__":
    check_singularities()