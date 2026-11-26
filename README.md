# MilkyWay 🐄🥛

**Autonomous Vision & Perception Pipeline for Robotic Cow Milking Simulation**

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/docs/harmonic)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-E95420.svg)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-Apache_2.0-green.svg)](LICENSE)

---

## Overview

**MilkyWay** is a complete, real-time computer vision and robotic milking simulation pipeline for automated dairy farming systems. Built with **ROS 2 Jazzy Jalisco** and **Gazebo Harmonic (Gz Sim 8)**, it features:
- **Full CAD Cow & Robotic Milker Arm Integration**: High-fidelity 3D Holstein cow mesh combined with a multi-DOF robotic milking arm carrying an industrial 4-cup milker cluster.
- **Anatomically Accurate 4-Quarter Dairy Cow Udder**: Modeled according to Holstein-Friesian veterinary standards with smooth abdominal suspension blend, independent fore/rear glandular quarters, central Median Suspensory cleft, and tapered teats with streak canal orifices.
- **Intel RealSense D435 RGB-D Simulation**: Native Gazebo Harmonic `rgbd_camera` generating realistic high-density point clouds bridged into ROS 2 via `ros_gz_bridge`.
- **6-DOF Pose & Approach Vector Estimation (PCA)**: Perception node using Principal Component Analysis on 3D point clusters to calculate the exact distal tip apex (<3mm) and the 3D cylinder insertion vector (quaternion) for robotic cup attachment.
- **Interactive 3D Visualizations**: RViz 2 markers showing live point clouds, teat tip spheres, 3D approach vector arrows, and real-time tilt angle tags.

---

## System Architecture

```
       +---------------------------------------------+
       |       Gazebo Harmonic (Gz Sim 8)            |
       |  - Cow Body, Hind Legs & Udder (4 Quarters) |
       |  - RealSense D435 RGB-D Camera Sensor       |
       +---------------------------------------------+
                              |
                              | gz.msgs.PointCloudPacked
                              v
       +---------------------------------------------+
       |               ros_gz_bridge                 |
       |  Bridges point clouds into ROS 2 Jazzy      |
       +---------------------------------------------+
                              |
                              | sensor_msgs/msg/PointCloud2
                              | Topic: /camera/depth/points
                              v
       +---------------------------------------------+
       |       teat_detector (Python Node)           |
       |  1. Pass-Through Filter (Udder Underside)   |
       |  2. Quadrant Partitioning (FL, FR, RL, RR)  |
       |  3. PCA Cylinder Axis (Eigen-Decomposition) |
       |  4. Exact Tip Apex Localization (<3mm)      |
       |  5. 6-DOF Quaternion & Tilt Angle Compute   |
       +---------------------------------------------+
                              |
         +--------------------+--------------------+
         |                                         |
         v                                         v
   /teat_poses                               /teat_markers
 (PoseArray / PoseStamped)                 (MarkerArray for RViz2)
 - 6-DOF Cup Targets                       - Spherical Tip Markers
 - Insertion Vectors (q)                   - 3D Approach Vector Arrows
 - Tilt Angles                             - Real-time Floating Text Tags
```

---

## Anatomical Udder Dimensions (Holstein Conformation)

* **Fore Quarters (FL, FR)**: $15.2\,\text{cm}$ inter-teat lateral spacing, $5.5\,\text{cm}$ barrel length, $2.4\,\text{cm}$ diameter, $4.6^\circ$ outward natural flare.
* **Rear Quarters (RL, RR)**: $11.6\,\text{cm}$ inter-teat lateral spacing, $4.8\,\text{cm}$ barrel length, $2.2\,\text{cm}$ diameter, $2.3^\circ$ inward inclination.
* **Longitudinal Distance**: $13.0\,\text{cm}$ front-to-rear separation.
* **Median Suspensory Ligament (MSL)**: Anatomical central cleft running longitudinally between left and right quarters.

---

## ROS 2 Topics & Interfaces

| Topic | Message Type | Description |
|---|---|---|
| `/camera/depth/points` | `sensor_msgs/msg/PointCloud2` | Raw 3D point cloud from RealSense D435 |
| `/teat_poses` | `geometry_msgs/msg/PoseArray` | 6-DOF poses of all 4 teat tips with approach orientations |
| `/teat_fl/pose` | `geometry_msgs/msg/PoseStamped` | 6-DOF target pose for Front-Left teat |
| `/teat_fr/pose` | `geometry_msgs/msg/PoseStamped` | 6-DOF target pose for Front-Right teat |
| `/teat_rl/pose` | `geometry_msgs/msg/PoseStamped` | 6-DOF target pose for Rear-Left teat |
| `/teat_rr/pose` | `geometry_msgs/msg/PoseStamped` | 6-DOF target pose for Rear-Right teat |
| `/teat_markers` | `visualization_msgs/msg/MarkerArray` | 3D visual markers (spheres, arrows, text labels) in RViz 2 |
| `/teat_centroid` | `geometry_msgs/msg/PointStamped` | Primary teat tip coordinates |

---

## Installation & Setup

### Prerequisites
* **OS**: Ubuntu 24.04 LTS (Noble Numbat)
* **ROS 2**: Jazzy Jalisco (`desktop` recommended)
* **Simulator**: Gazebo Harmonic (`ros_gz`)

```bash
# Install core ROS 2 Jazzy & Gazebo Harmonic dependencies
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-sensor-msgs-py \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  python3-scipy \
  python3-numpy
```

### Build the Workspace

```bash
# Clone the repository
git clone https://github.com/akhiljithvg/milkyway.git ~/milker_ws
cd ~/milker_ws

# Source ROS 2 underlay
source /opt/ros/jazzy/setup.bash

# Build the packages
colcon build --packages-select milker_vision

# Source the workspace overlay
source install/setup.bash
```

---

## Running the Simulation

### 1. Launch Gazebo Harmonic + Perception + RViz 2 Dashboard

```bash
source install/setup.bash
ros2 launch milker_vision vision_sim.launch.py rviz:=true
```

### 2. Headless Mode (for Cloud, CI/CD, or Remote SSH)

```bash
ros2 launch milker_vision vision_sim.launch.py gz_args:='-r -s empty.sdf'
```

### 3. Verify Active Targets

In a second terminal:

```bash
source install/setup.bash

# Echo the live 6-DOF poses of all 4 teats
ros2 topic echo /teat_poses

# Echo Front-Left teat pose
ros2 topic echo /teat_fl/pose
```

---

## Repository Structure

```
milkyway/
├── .gitignore
├── README.md
└── src/
    ├── milker_sim/                     # Milking system packages
    └── milker_vision/
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/
        │   └── milker_tracking.rviz    # Pre-configured RViz 2 dashboard
        ├── launch/
        │   └── vision_sim.launch.py    # Master launch orchestrator
        ├── scripts/
        │   └── teat_detector.py        # 6-DOF PCA perception node
        └── urdf/
            ├── assets/                 # CAD meshes (.stl) for cow & robot arm
            ├── robot.urdf              # Standalone compiled URDF model
            └── vision_rig.xacro        # Master merged parametric XACRO model
```

---

## License

Licensed under the [Apache License, Version 2.0](LICENSE).
