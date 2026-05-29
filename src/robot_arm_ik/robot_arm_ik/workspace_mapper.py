import numpy as np
import matplotlib.pyplot as plt
from five_bar_kinematics import FiveBarKinematics

def map_workspace():
    kin = FiveBarKinematics()
    
    # Create a grid to test (from X: -600 to 600, Y: -200 to 800)
    # We test a point every 10 millimeters
    x_vals = np.arange(-600, 600, 5)
    y_vals = np.arange(-800, 800, 5)
    
    reachable_x = []
    reachable_y = []
    
    print(f"Scanning {len(x_vals) * len(y_vals)} coordinates...")

    # Interrogate the IK solver for every single point on the grid
    for x in x_vals:
        for y in y_vals:
            # If our solver returns solutions, the point is mathematically reachable
            if kin.ik_5bar(x, y) is not None:
                reachable_x.append(x)
                reachable_y.append(y)

    print(f"Found {len(reachable_x)} reachable points. Plotting...")

    # Plotting the results
    plt.figure(figsize=(10, 8))
    plt.scatter(reachable_x, reachable_y, c='blue', s=1, alpha=0.5, label='Reachable Workspace')
    
    # Plot the motor locations for visual reference
    plt.plot(kin.motor_a_x, kin.motor_y, 'ro', markersize=8, label='Motor A (Left)')
    plt.plot(kin.motor_b_x, kin.motor_y, 'ro', markersize=8, label='Motor B (Right)')
    
    plt.title('Mathematical Workspace of Custom 5-Bar Linkage')
    plt.xlabel('X coordinate (mm)')
    plt.ylabel('Y coordinate (mm)')
    plt.axis('equal')  # Ensures 1mm in X looks the same as 1mm in Y
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == '__main__':
    map_workspace()