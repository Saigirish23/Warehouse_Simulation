# Warehouse_Simulation

Minimal ROS 2 workspace for warehouse mapping and navigation simulation.

Purpose
- Run Gazebo warehouse simulation, create maps (SLAM) and run localization + navigation with keepout mask.

Prerequisites
- ROS 2 Humble installed and sourced (`/opt/ros/humble/setup.bash`)
- Gazebo (compatible with ROS 2 Humble) and `ros-humble-gazebo-ros-pkgs`
- Nav2 packages: `ros-humble-navigation2`, `ros-humble-nav2-bringup`
- Teleop: `ros-humble-teleop-twist-keyboard`
- Colcon build tools: `python3-colcon-common-extensions`

Build
```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-gazebo-ros-pkgs ros-humble-teleop-twist-keyboard \
  python3-colcon-common-extensions

source 
cd ~/Warehouse_Simulation
colcon build --symlink-install --packages-select scuttle_gazebo_ros2 scuttle_navigation2 scuttle_description_ros2
source ~/Warehouse_Simulation/install/setup.bash
Terminal Commands to 
PHASE 1 — MAPPING (terminals)

Terminal 1 (Gazebo)
cd ~/Warehouse_Simulation
source install/setup.bash
ros2 launch scuttle_gazebo_ros2 warehouse_launch.py

Terminal 2 (SLAM)
cd ~/Warehouse_Simulation
source install/setup.bash
ros2 launch scuttle_navigation2 online_async_launch.py

Terminal 3 (Teleop)
cd ~/Warehouse_Simulation
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

Terminal 4 (Geotag)
cd ~/Warehouse_Simulation
source install/setup.bash
python3 src/scuttle_navigation2/launch/geotag_recorder.py

Terminal 5: RVIZ
cd ~/Warehouse_Simulation
source install/setup.bash
rviz2 -d src/scuttle_navigation2/config/nav2_scuttle.rviz

PHASE 2 — NAVIGATION
Terminal 1: GAZEBO
cd ~/Warehouse_Simulation
source install/setup.bash
ros2 launch scuttle_gazebo_ros2 warehouse_launch.py

Terminal 2: LOCALIZATION (MAP SERVER + AMCL) with keepout
cd ~/Warehouse_Simulation
source install/setup.bash
ros2 launch scuttle_navigation2 localization_keepout_launch.py \
  map:=/home/b24me1066/Warehouse_Simulation/maps/scuttle_slam_map.yaml \
  keepout_mask:=/home/b24me1066/Warehouse_Simulation/maps/keepout_mask.yaml
# if lifecycle activation needed:
# ros2 lifecycle set /keepout_filter_mask_server activate

Terminal 3: RVIZ (set initial pose)
cd ~/Warehouse_Simulation
source install/setup.bash
rviz2 -d src/scuttle_navigation2/config/nav2_scuttle.rviz

Terminal 4: NAVIGATION
cd ~/Warehouse_Simulation
source install/setup.bash
ros2 launch scuttle_navigation2 navigation_launch.py

Terminal 5: WAYPOINT NAVIGATION
cd ~/Warehouse_Simulation
source install/setup.bash
python3 ~/Warehouse_Simulation/src/scuttle_navigation2/launch/waypoint_navigator.py

