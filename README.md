# Robotic Arm — Mechanical Design → Embedded Firmware → ROS 2 Motion Planning

![ROS2](https://img.shields.io/badge/ROS2-Humble%2FIron-22314E?logo=ros&logoColor=white)
![MoveIt](https://img.shields.io/badge/MoveIt2-Motion%20Planning-blue)
![Python](https://img.shields.io/badge/Python-numpy%20%7C%20rclpy-3776AB?logo=python&logoColor=white)
![C++](https://img.shields.io/badge/C%2B%2B-ros2__control-00599C?logo=cplusplus&logoColor=white)
![ESP32](https://img.shields.io/badge/Firmware-ESP32%20%2F%20Arduino-E7352C?logo=espressif&logoColor=white)

A self-designed 2-DOF five-bar linkage robotic arm, built as a full-stack robotics project: CAD → 3D-printed/machined hardware → custom embedded firmware → a ROS 2 + MoveIt 2 motion planning and control stack, connected over a hand-rolled binary serial protocol.

This isn't a simulation-only or tutorial-following project — every layer (mechanical, firmware, kinematics, planning, and the ros2_control integration) is custom-built for this specific arm's geometry and hardware.

## Highlights

- **Mechanical design:** Custom five-bar (parallel-linkage) arm designed in CAD from scratch — full STEP/STL source in [`cad/`](./cad)
- **Closed-form kinematics from first principles:** Derived and implemented forward and inverse kinematics for a five-bar linkage, including all 4 valid IK branches and transmission-angle singularity detection (`five_bar_kinematics.py`)
- **Motion planning:** An A* planner that searches over `(x, y, kinematic-branch)` states, costing branch switches and validating every candidate against MoveIt 2's live collision-checking service (`astar_planner.py`)
- **MoveIt 2 integration:** Full MoveIt Setup Assistant configuration (SRDF, kinematics, joint limits, RViz) driving real-time collision validation
- **`ros2_control` hardware plugin written in C++:** A `SystemInterface` plugin with a ring-buffer serial parser, custom packet framing, and lifecycle management (`arm_hardware_interface.cpp`)
- **Embedded firmware:** ESP32/Arduino firmware driving two closed-loop stepper drivers via `FastAccelStepper`, with fault/alarm handling and 50 Hz telemetry feedback
- **Designed a binary wire protocol** for low-latency, low-overhead host↔microcontroller communication (see [Serial Protocol](#serial-protocol))
- ~1,900 lines of original Python/C++/firmware code across planning, control, and embedded layers

## System Architecture

```
                     /target_pose (x, y)
                             │
                             ▼
                 ┌──────────────────────┐        /check_state_validity
                 │   robot_arm_ik       │◄───────────────────────────────┐
                 │   (ik_node.py)       │                                │
                 └─────────┬────────────┘                        ┌───────────────┐
                           │ plan request                        │    MoveIt 2   │
                           ▼                                     │  (collision   │
                 ┌──────────────────────┐                        │   checking)   │
                 │   A* Planner         │────────────────────────►               │
                 │ (branch + collision  │                        └───────────────┘
                 │  aware search)       │
                 └─────────┬────────────┘
                           │ synchronized motion frames (~100 Hz)
                           ▼
                 ┌─────────────────────┐   18-byte binary packets   ┌─────────────┐
                 │  Serial Link (UART  │───────────────────────────►│   ESP32     │
                 │  115200 baud        │◄───────────────────────────│  Firmware   │
                 └─────────────────────┘   11-byte telemetry (50Hz) └──────┬──────┘
                                                                             │
                                                                    FastAccelStepper
                                                                             │
                                                                   ┌─────────▼─────────┐
                                                                   │ 2× Stepper Motors │
                                                                   │ (T60 drivers)     │
                                                                   └───────────────────┘
```

## Repository Structure

```
Robotic-Arm/
├── cad/                          # Original CAD (STEP + STL) for the five-bar arm
├── firmware/esp_microros/        # ESP32 firmware — stepper control + binary serial protocol
├── src/
│   ├── arm_bringup/              # Top-level launch file for the full stack
│   ├── arm_commander/            # High-level command API (in progress)
│   ├── arm_hardware_interface/   # ros2_control C++ plugin + URDF/xacro + visual meshes
│   ├── arm_moveit_config/        # MoveIt 2 config (SRDF, kinematics, RViz, demo launch)
│   └── robot_arm_ik/             # Kinematics, planning, and the main control node
└── .gitignore
```

## How It Works

1. A target end-effector position is published on **`/target_pose`**.
2. The **A\* planner** searches the workspace, switching between the linkage's four kinematic branches when needed, validating each state live against MoveIt's collision service.
3. The resulting path is interpolated at ~100 Hz into per-tick motor angle/speed commands.
4. Commands are packed into compact binary frames and streamed to the ESP32 over serial.
5. The **firmware** drives both steppers via `FastAccelStepper`, monitors driver fault lines, and reports back live motor position at 50 Hz.
6. Live joint states are republished for RViz visualization and closed-loop tracking.

The same wire protocol is also exposed as a standard **`ros2_control`** hardware interface, so the arm can be driven through ROS 2's conventional controller pipeline (`position_controller` / `joint_state_broadcaster`) as well as the direct planning path.

## Tech Stack

| Layer | Tools |
|---|---|
| Mechanical | CAD (STEP/STL export) |
| Firmware | C++ (Arduino/ESP32), FastAccelStepper |
| Low-level control | C++, `ros2_control`, `pluginlib`, POSIX serial (termios) |
| Kinematics & Planning | Python, NumPy, custom A* search |
| Motion Planning Framework | ROS 2, MoveIt 2 |
| Communication | Custom binary serial protocol over UART |

## Serial Protocol

Designed for minimal latency and overhead between the host and the ESP32:

| Direction | Size | Layout |
|---|---|---|
| Host → ESP32 (command) | 18 bytes | `0xAA` \| `motor_a_rad` (f32) \| `motor_b_rad` (f32) \| `motor_a_hz` (f32) \| `motor_b_hz` (f32) \| `0xBB` |
| ESP32 → Host (telemetry) | 11 bytes | `0xCC` \| `status` (u8) \| `current_a_rad` (f32) \| `current_b_rad` (f32) \| `0xDD` |

## Getting Started

```bash
# 1. Build the ROS 2 workspace
colcon build
source install/setup.bash

# 2. Flash firmware/esp_microros/esp_microros.ino to the ESP32
#    (Arduino IDE or PlatformIO, with FastAccelStepper installed)

# 3. Launch the full stack
ros2 launch arm_bringup arm.launch.py

# 4. Send a target position
ros2 topic pub /target_pose geometry_msgs/msg/Point "{x: 0.0, y: 400.0}" --once

# 5. Visualize in RViz (optional)
ros2 launch arm_moveit_config demo.launch.py
```

## Roadmap

- [ ] High-level command API in `arm_commander`
- [ ] Trajectory blending for smoother multi-point paths
- [ ] Camera-based perception for closed-loop pick-and-place

## About Me

Built by **Shashank Vijavargia** — mechanical design, embedded firmware, and ROS 2 software all done end-to-end for this project.

*Add your LinkedIn / portfolio site / email here so recruiters can reach you.*
