# 🏭 Warehouse Simulation

![ROS2](https://img.shields.io/badge/ROS%202-Humble-blue?style=for-the-badge&logo=ros)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?style=for-the-badge&logo=ubuntu)
![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-orange?style=for-the-badge)
![Nav2](https://img.shields.io/badge/Nav2-Navigation-green?style=for-the-badge)
![Python](https://img.shields.io/badge/Python-3.10-yellow?style=for-the-badge&logo=python)

A ROS 2 and Gazebo-based warehouse simulation for mapping, localization, and autonomous navigation using the Nav2 stack with a Scuttle robot.

---

## 📌 Overview

This repository demonstrates a complete autonomous mobile robot workflow inside a simulated warehouse environment.

The project includes:

- 🏭 Warehouse simulation in Gazebo
- 🗺️ SLAM-based map generation
- 📍 Localization using AMCL
- 🧭 Autonomous navigation using Nav2
- 🚧 Keepout mask support for restricted zones
- 🎯 Waypoint-based navigation
- 🖥️ RViz visualization
- 🎮 Keyboard teleoperation

---

## ✨ Features

- 🤖 Scuttle robot simulation
- 🗺️ Real-time warehouse mapping
- 📡 Nav2 navigation stack integration
- 🚧 Keepout zone filtering
- 🎯 Autonomous waypoint navigation
- 🖥️ Preconfigured RViz setup
- 🎮 Manual teleoperation support
- 🧩 Modular ROS 2 workspace structure

---

## 🛠️ Tech Stack

| Tool / Framework | Purpose |
|---|---|
| ROS 2 Humble | Robot middleware |
| Gazebo | Warehouse simulation |
| Nav2 | Autonomous navigation |
| SLAM Toolbox | Mapping |
| RViz2 | Visualization |
| Python | Helper scripts and waypoint navigation |
| Colcon | ROS 2 workspace build tool |

---

## 📂 Repository Structure

```bash
Warehouse_Simulation/
├── maps/
│   ├── scuttle_slam_map.yaml
│   └── keepout_mask.yaml
│
├── src/
│   ├── scuttle_description_ros2/
│   ├── scuttle_gazebo_ros2/
│   └── scuttle_navigation2/
│
├── build/
├── install/
└── log/
```

---

## 💻 System Requirements

- **Ubuntu 22.04**
- **ROS 2 Humble**
- **Gazebo Classic**
- **RViz2**
- **Python 3.10+**

---

## 📦 Dependencies

Install the required packages:

```bash
sudo apt update

sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-teleop-twist-keyboard \
  ros-humble-slam-toolbox \
  python3-colcon-common-extensions
```

---

## ⚙️ Installation and Build

```bash
cd ~

git clone https://github.com/Saigirish23/Warehouse_Simulation

cd Warehouse_Simulation

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---

## 🗺️ Phase 1: Mapping

Use this phase to generate the warehouse map using SLAM.

### 🚀 Terminal 1: Launch Gazebo

```bash
cd ~/Warehouse_Simulation
source install/setup.bash

ros2 launch scuttle_gazebo_ros2 warehouse_launch.py
```

### 🧠 Terminal 2: Start SLAM

```bash
cd ~/Warehouse_Simulation
source install/setup.bash

ros2 launch scuttle_navigation2 online_async_launch.py
```

### 🎮 Terminal 3: Teleoperation

```bash
cd ~/Warehouse_Simulation
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the keyboard to move the robot and explore the warehouse.

### 📍 Terminal 4: Geotag Recorder

```bash
cd ~/Warehouse_Simulation
source install/setup.bash

python3 src/scuttle_navigation2/launch/geotag_recorder.py
```

### 🖥️ Terminal 5: RViz

```bash
cd ~/Warehouse_Simulation
source install/setup.bash

rviz2 -d src/scuttle_navigation2/config/nav2_scuttle.rviz
```

### 💾 Save the Map

After mapping the warehouse, save the generated map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/Warehouse_Simulation/maps/scuttle_slam_map
```

The map files will be saved inside the `maps/` directory.

---

## 🧭 Phase 2: Navigation

Use this phase to run localization and autonomous navigation.

### 🚀 Terminal 1: Launch Gazebo

```bash
cd ~/Warehouse_Simulation
source install/setup.bash

ros2 launch scuttle_gazebo_ros2 warehouse_launch.py
```

### 📡 Terminal 2: Localization with Keepout Mask

```bash
cd ~/Warehouse_Simulation
source install/setup.bash

ros2 launch scuttle_navigation2 localization_keepout_launch.py \
  map:=~/Warehouse_Simulation/maps/scuttle_slam_map.yaml \
  keepout_mask:=~/Warehouse_Simulation/maps/keepout_mask.yaml
```

If lifecycle activation is required:

```bash
ros2 lifecycle set /keepout_filter_mask_server activate
```

### 🖥️ Terminal 3: RViz

```bash
cd ~/Warehouse_Simulation
source install/setup.bash

rviz2 -d src/scuttle_navigation2/config/nav2_scuttle.rviz
```

> Use the **2D Pose Estimate** tool in RViz to set the robot’s initial pose before starting navigation.

### 🧭 Terminal 4: Start Navigation

```bash
cd ~/Warehouse_Simulation
source install/setup.bash

ros2 launch scuttle_navigation2 navigation_launch.py
```

### 🎯 Terminal 5: Waypoint Navigation

```bash
cd ~/Warehouse_Simulation
source install/setup.bash

python3 ~/Warehouse_Simulation/src/scuttle_navigation2/launch/waypoint_navigator.py
```

The robot will follow predefined waypoints inside the warehouse.

---

## 🖼️ Visualization

The RViz configuration includes:

- 🤖 Robot model
- 🗺️ Occupancy grid map
- 📡 Laser scan data
- 🧭 Planned path
- 🚧 Local and global costmaps
- 🔗 TF frames
🔗 TF frames
