# MilkyWay 🐄🥛

**Two-Phase Autonomous Vision & Robotic Milking Simulation (ROS 2 Jazzy + Gazebo Harmonic)**

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/docs/harmonic)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-E95420.svg)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-Apache_2.0-green.svg)](LICENSE)

---

## Overview

**MilkyWay** is an end-to-end autonomous robotic cow milking simulation built on **ROS 2 Jazzy Jalisco** and **Gazebo Harmonic (Gz Sim 8)**. It combines a full CAD Holstein cow model, an anatomically accurate 4-quarter bovine udder, a 7-DOF robotic milking arm with a 4-cup cluster, and a **two-camera hierarchical vision pipeline**:

1. **Overhead RGB-D Camera** (`overhead_camera_link` at `Z = 2.30 m`): Scans the cow from above, draws virtual vertical/horizontal crosshair reference lines over the spine center, estimates the udder location using bovine anatomical offsets, and **locks** the coarse 3D target waypoint.
2. **Arm-Mounted End-Effector RGB-D Camera** (`ee_camera_link` mounted between the 4 suction cups on `part_2_3`, `135°` FOV): Activates only after the arm reaches a safe clearance below the udder, scans upward from below, confirms all 4 distal teat tips over 8 consecutive stable frames, and guides **slow, accurate sequential cup docking** (`FL → FR → RL → RR`).

---

## 5-Step Autonomous Milking Sequence

1. **Step 1 — Overhead Cow Detection & Udder Lock (`overhead_detector.py`)**
   - Subscribes to `/overhead_camera/depth/points` looking straight down at the cow's back.
   - Filters out the floor and side-mounted robot gantry (`Y ∈ [-0.25, 0.25]`, `Z ∈ [0.70, 1.80]`) to isolate the cow torso and spine center.
   - Publishes virtual crosshair lines, cow bounding box, and estimated udder sphere on `/overhead_markers`.
   - Applies anatomical offsets (`+0.16 m` X, `0.0 m` Y, `-0.94 m` Z from spine) and latches `/udder_position_locked = True` once stable within `2 cm` for `10` consecutive frames (`world [0.516, 0.000, 0.497]`).

2. **Step 2 — Safe-Clearance Arm Approach (`arm_controller.py`: `IDLE → APPROACHING → POSITIONED`)**
   - Uses exact 2-DOF planar analytical inverse kinematics (pivot `(0.100, -0.408)`, link length `L = 0.5188 m`, `phi0 = -0.0627 rad`) to swing the 4-cup cluster from its parked side position directly beneath the locked udder waypoint.
   - Moves slowly and smoothly via slew-rate limiting (`0.08 m/s` linear, `0.24 rad/s` angular) while keeping `part_4` nested `+0.15 m` inside `part_4__3` and maintaining `safe Z = 0.249 m` (~4 cm below the teat tips and clear of the ground).

3. **Step 3 — End-Effector Camera Activation (`ee_teat_detector.py`)**
   - Remains in standby during `IDLE` and `APPROACHING`.
   - Automatically activates when `/arm_state` transitions to `POSITIONED`, scanning upward from beneath the udder.

4. **Step 4 — Close-Range Teat Tip Extraction & 8-Frame Confirmation (`ee_teat_detector.py`)**
   - Isolates the closest protruding depth layer (`depth ∈ [0.015, 0.12] m`, `protrusion_slice = 0.028 m`) to extract only the distal teat tips without udder lobe contamination.
   - Transforms points into the `world` frame via TF2, partitions into 4 anatomical quadrants (`FL`, `FR`, `RL`, `RR`) using bounding-box midpoints (with a minimum `6 cm` span check to prevent single-teat collapse), and extracts each distal tip apex.
   - Confirms stability across `8` consecutive frames (`< 1.2 cm` drift) before publishing `/teat_tips_confirmed = True`.

5. **Step 5 — Fine Alignment & Sequential 4-Cup Docking (`arm_controller.py`)**
   - Transitions `POSITIONED → FINE_ALIGN` to center the cluster under the confirmed 4-teat centroid, locking the 4 teat targets (`self.dock_targets`).
   - Executes two-subphase (`ALIGN → LIFT`) sequential docking:
     `DOCKING_FL → DOCKING_FR → DOCKING_RL → DOCKING_RR → MILKING → RETRACTING`.
   - Updates `/docking_status` and turns each cup marker green in `/docking_markers` as it docks.

---

## System Architecture

```
+-----------------------------------------------------------------------+
|                   Gazebo Harmonic (Gz Sim 8, OGRE2)                   |
|  - Holstein Cow Mesh + 4-Quarter Anatomical Udder                     |
|  - 7-DOF Milking Arm + JointPositionControllers + JointStatePublisher |
|  - Overhead RGB-D Camera (Z=2.30m) & EE RGB-D Camera (on part_2_3)    |
+-----------------------------------------------------------------------+
         |                                  |                 ^
         | gz.msgs.PointCloudPacked         | gz.msgs.Model   | gz.msgs.Double
         v                                  v                 |
+-----------------------------------------------------------------------+
|                             ros_gz_bridge                             |
|  /overhead_camera/depth/points   /ee_camera/depth/points              |
|  /joint_states                   /arm/<joint_name>/cmd_pos            |
+-----------------------------------------------------------------------+
         |                                  |                 ^
         v                                  v                 |
+------------------------+       +------------------------+   |
|  overhead_detector.py  |       |   ee_teat_detector.py  |   |
|  - Cow spine centroid  |       |  - Activates on        |   |
|  - Crosshair markers   |       |    POSITIONED state    |   |
|  - Locks udder XYZ     |       |  - 8-frame 4-teat tip  |   |
+------------------------+       |    confirmation (TF2)  |   |
         |                       +------------------------+   |
         | /estimated_udder_position        |                 |
         | /udder_position_locked           | /ee_teat_*/pose |
         v                                  v /teat_tips_confirmed
+-----------------------------------------------------------------------+
|                          arm_controller.py                            |
|  IDLE -> APPROACHING -> POSITIONED -> FINE_ALIGN ->                   |
|  DOCKING_FL -> DOCKING_FR -> DOCKING_RL -> DOCKING_RR ->              |
|  MILKING -> RETRACTING  (Slew-rate limited, 20 Hz control loop)       |
+-----------------------------------------------------------------------+
                                   |
                                   v
                            RViz 2 Dashboard
        (/overhead_markers, /ee_teat_markers, /docking_markers)
```

---

## Dual-Camera & 7-DOF Arm Hardware

### Cameras (2 Sensors Only)
| Camera | Link Frame | Mount Pose / Parent | Resolution & FOV | Clip Range |
|---|---|---|---|---|
| **Overhead RGB-D** | `overhead_camera_link` | `world` at `(0.35, 0.0, 2.30)`, pitched `+90°` down | `640×480`, `1.047 rad` (`60°`) | `0.2 – 4.0 m` |
| **End-Effector RGB-D** | `ee_camera_link` | `part_2_3` at `(0.025, 0.025, 0.115)`, pitched `-90°` up | `320×320`, `2.35 rad` (`135°`) | `0.01 – 0.40 m` |

### 7-DOF Arm Joint Chain
1. `base_to_part_2` (Prismatic Y, `[-1.00, 0.80] m`, `effort=300 N`) — longitudinal gantry along cow
2. `part_2_to_part_3` (Prismatic Z, `[-0.40, 0.30] m`, `effort=500 N`) — main vertical carriage supporting the `8.62 kg` lower arm
3. `part_3_to_part_4__3` (Revolute X, `[-0.7854, 0.7854] rad`, `effort=200 Nm`) — tube tilt
4. `part_4__3_to_part_4` (Prismatic Z, `[0.05, 0.35] m`, `initial=0.15 m`, `effort=300 N`) — telescoping rod kept nested `+15 cm` inside `part_4__3`
5. `part_4_to_part_4__2` (Revolute Z, `[-3.1416, 3.1416] rad`, `effort=100 Nm`) — under-cow swing yaw
6. `part_4__2_to_part_5` (Revolute X, `[-1.5708, 1.5708] rad`, `effort=200 Nm`) — horizontal bar pitch
7. `part_5_to_part_2_3` (Revolute Z, `[-1.5708, 1.5708] rad`, `effort=20 Nm`) — 4-cup cluster wrist yaw

---

## Anatomical Udder & Verified Tip Accuracy

* **Fore Quarters (`FL`, `FR`)**: $15.0\,\text{cm}$ lateral spacing, $5.5\,\text{cm}$ barrel length, $1.2\,\text{cm}$ radius.
* **Rear Quarters (`RL`, `RR`)**: $11.0\,\text{cm}$ lateral spacing, $4.8\,\text{cm}$ barrel length, $1.1\,\text{cm}$ radius.
* **Longitudinal Separation**: $13.5\,\text{cm}$ front-to-rear.

Confirmed end-effector depth detection vs. URDF ground truth:

| Teat | URDF Ground Truth `(X, Y, Z)` m | Confirmed EE Camera `(X, Y, Z)` m |
|---|---|---|
| `FL` (Front-Left) | `[0.430, +0.075, 0.288]` | `[0.436, +0.070, 0.294]` |
| `FR` (Front-Right) | `[0.430, -0.075, 0.288]` | `[0.434, -0.073, 0.291]` |
| `RL` (Rear-Left) | `[0.565, +0.055, 0.296]` | `[0.564, +0.052, 0.298]` |
| `RR` (Rear-Right) | `[0.565, -0.055, 0.296]` | `[0.564, -0.053, 0.298]` |

---

## ROS 2 Topics & Interfaces

| Topic | Message Type | Direction / Description |
|---|---|---|
| `/overhead_camera/depth/points` | `sensor_msgs/msg/PointCloud2` | Overhead RGB-D point cloud from Gazebo |
| `/ee_camera/depth/points` | `sensor_msgs/msg/PointCloud2` | Close-range upward-looking point cloud from milker cluster |
| `/estimated_udder_position` | `geometry_msgs/msg/PointStamped` | Estimated udder center in `world` frame |
| `/udder_position_locked` | `std_msgs/msg/Bool` | Latches `True` once overhead udder estimate stabilizes |
| `/overhead_markers` | `visualization_msgs/msg/MarkerArray` | Overhead crosshair lines, cow bbox, and udder target sphere |
| `/ee_teat_poses` | `geometry_msgs/msg/PoseArray` | All 4 detected teat tip poses in `world` frame |
| `/ee_teat_{fl,fr,rl,rr}/pose` | `geometry_msgs/msg/PoseStamped` | Individual teat tip target poses (`world` frame) |
| `/teat_tips_confirmed` | `std_msgs/msg/Bool` | `True` after 8 consecutive stable frames of all 4 teats |
| `/ee_teat_markers` | `visualization_msgs/msg/MarkerArray` | 3D spheres at confirmed distal teat tips |
| `/arm_state` | `std_msgs/msg/String` | Current state machine state (`IDLE` → `MILKING` → `RETRACTING`) |
| `/docking_status` | `std_msgs/msg/String` | Live cup docking progress (`Docked=['FL', ...]`) |
| `/docking_markers` | `visualization_msgs/msg/MarkerArray` | Suction cup status cylinders (gray → green when docked) |
| `/joint_states` | `sensor_msgs/msg/JointState` | Live 7-DOF arm joint positions from Gazebo |
| `/arm/<joint_name>/cmd_pos` | `std_msgs/msg/Float64` | Position commands bridged to Gazebo joint controllers |

---

## Installation & Build

### Prerequisites
* **OS**: Ubuntu 24.04 LTS (Noble Numbat)
* **ROS 2**: Jazzy Jalisco
* **Simulator**: Gazebo Harmonic (`ros_gz`)

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-sensor-msgs-py \
  ros-jazzy-tf2-ros \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  python3-scipy \
  python3-numpy
```

### Build the Workspace

```bash
git clone https://github.com/akhiljithvg/milkyway.git ~/milker_ws
cd ~/milker_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select milker_vision
source install/setup.bash
```

---

## Running the Simulation

### 1. Full Simulation with Gazebo Harmonic + RViz 2 Dashboard
```bash
cd ~/milker_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch milker_vision vision_sim.launch.py rviz:=true
```

### 2. Headless Mode (CI / Remote SSH)
```bash
ros2 launch milker_vision vision_sim.launch.py gz_args:='-r -s empty.sdf'
```

### 3. Monitor Live Pipeline State
In a second terminal:
```bash
source /opt/ros/jazzy/setup.bash
source ~/milker_ws/install/setup.bash

ros2 topic echo /arm_state
ros2 topic echo /docking_status
ros2 topic echo /ee_teat_poses
```

---

## Repository Structure

```
milkyway/
├── .gitignore
├── README.md
└── src/
    ├── milker_sim/                         # Auxiliary simulation package
    └── milker_vision/
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/
        │   └── milker_tracking.rviz        # RViz 2 config (point clouds + markers)
        ├── launch/
        │   └── vision_sim.launch.py        # Master Gazebo + ROS 2 bridge + pipeline launch
        ├── scripts/
        │   ├── overhead_detector.py        # Phase 1: Overhead cow detection & udder lock
        │   ├── ee_teat_detector.py         # Phase 2: Close-range EE 4-teat tip confirmation
        │   ├── arm_controller.py           # Phase 3: 7-DOF IK & sequential cup docking
        │   └── teat_detector.py            # Legacy single-camera PCA reference node
        └── urdf/
            ├── assets/                     # CAD meshes (.stl) for cow & robotic arm
            ├── robot.urdf                  # Compiled URDF generated from XACRO
            └── vision_rig.xacro            # Parametric XACRO (cow, udder, arm, 2 cameras, PIDs)
```

---

## License

Licensed under the [Apache License, Version 2.0](LICENSE).
