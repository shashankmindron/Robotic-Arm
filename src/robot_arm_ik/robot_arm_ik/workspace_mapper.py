import numpy as np
import matplotlib.pyplot as plt
import math

def map_workspace():
    print("Generating 5-Bar Linkage Workspace Maps...")
    
    # Hardware Configuration
    d  = 350.0
    l1 = 200.0
    l2 = 200.0
    l3 = 290.0
    l4 = 290.0
    motor_a_x = -d / 2.0
    motor_b_x =  d / 2.0
    motor_y   = 0.0
    sing_threshold = math.radians(5.0)

    # Grid Setup (Scanning 10mm increments to generate the map quickly)
    x_vals = np.arange(-450, 450, 5)
    y_vals = np.arange(-600, 600, 5)

    # Dictionaries to hold plotting coordinates for each branch
    # Format: {'branch_name': {'safe_x': [], 'safe_y': [], 'sing_x': [], 'sing_y': []}}
    branches = {
        'out_out': {'sx': [], 'sy': [], 'dx': [], 'dy': []},
        'in_in':   {'sx': [], 'sy': [], 'dx': [], 'dy': []},
        'out_in':  {'sx': [], 'sy': [], 'dx': [], 'dy': []},
        'in_out':  {'sx': [], 'sy': [], 'dx': [], 'dy': []}
    }

    for x in x_vals:
        for y in y_vals:
            # --- Type I Check (Can the arms even reach this point?) ---
            dx_a, dy_a = x - motor_a_x, y - motor_y
            R_a = np.hypot(dx_a, dy_a)
            if R_a > (l1 + l3) or R_a < abs(l1 - l3): continue

            dx_b, dy_b = x - motor_b_x, y - motor_y
            R_b = np.hypot(dx_b, dy_b)
            if R_b > (l2 + l4) or R_b < abs(l2 - l4): continue

            # --- Calculate Angles ---
            cos_a = np.clip((l1**2 + R_a**2 - l3**2) / (2.0 * l1 * R_a), -1.0, 1.0)
            cos_b = np.clip((l2**2 + R_b**2 - l4**2) / (2.0 * l2 * R_b), -1.0, 1.0)
            alpha_a, alpha_b = np.arccos(cos_a), np.arccos(cos_b)
            base_a, base_b = np.arctan2(dy_a, dx_a), np.arctan2(dy_b, dx_b)

            th_a_out, th_a_in = base_a + alpha_a, base_a - alpha_a
            th_b_out, th_b_in = base_b - alpha_b, base_b + alpha_b

            permutations = {
                'out_out': (th_a_out, th_b_out),
                'in_in':   (th_a_in, th_b_in),
                'out_in':  (th_a_out, th_b_in),
                'in_out':  (th_a_in, th_b_out)
            }

            for name, (th_a, th_b) in permutations.items():
                # Find Elbows
                elbow_a_x = motor_a_x + l1 * math.cos(th_a)
                elbow_a_y = motor_y   + l1 * math.sin(th_a)
                elbow_b_x = motor_b_x + l2 * math.cos(th_b)
                elbow_b_y = motor_y   + l2 * math.sin(th_b)

                # Baseplate Collision Guard (If Distal A hits the motor mounts / crosses Y=0)
                # You mentioned this is your primary collision constraint!
                if elbow_a_y < 0 or y < 0:
                    continue # Discard this point entirely for this branch

                # Singularity Math (Transmission Angle)
                distal_a_ang = math.atan2(y - elbow_a_y, x - elbow_a_x)
                distal_b_ang = math.atan2(y - elbow_b_y, x - elbow_b_x)
                angle_diff = abs(distal_a_ang - distal_b_ang) % math.pi

                if angle_diff < sing_threshold or abs(angle_diff - math.pi) < sing_threshold:
                    branches[name]['dx'].append(x) # DANGER ZONE
                    branches[name]['dy'].append(y)
                else:
                    branches[name]['sx'].append(x) # SAFE ZONE
                    branches[name]['sy'].append(y)

    # --- Plotting the Data ---
    fig, axs = plt.subplots(2, 2, figsize=(10, 10))
    fig.suptitle('5-Bar Workspace Map by Kinematic Branch', fontsize=16)
    
    plot_map = [
        (axs[0, 0], 'out_out'), (axs[0, 1], 'in_in'),
        (axs[1, 0], 'out_in'),  (axs[1, 1], 'in_out')
    ]

    for ax, name in plot_map:
        ax.set_title(f"Branch: {name}")
        ax.set_xlim(-450, 450)
        ax.set_ylim(-600, 600)
        ax.set_aspect('equal')
        ax.grid(True, linestyle='--', alpha=0.5)
        
        # Plot Safe Zones (Green)
        ax.scatter(branches[name]['sx'], branches[name]['sy'], c='g', s=2, label='Safe', alpha=0.6)
        # Plot Dead Zones (Red)
        ax.scatter(branches[name]['dx'], branches[name]['dy'], c='r', s=5, label='Singularity')
        # Plot Motors (Black)
        ax.scatter([motor_a_x, motor_b_x], [motor_y, motor_y], c='k', s=50, marker='s', label='Motors')

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    map_workspace()