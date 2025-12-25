# Warehouse_Simulation
Purpose: “ROS 2 Gazebo-based warehouse simulation for mapping and navigation with Nav2 (scuttle robot).”
​
Prerequisites:
Ubuntu version, ROS 2 distro (e.g. Humble/Foxy), Gazebo / Ignition version.
​
Required system packages (nav2, gazebo_ros, slam_toolbox or equivalent, teleop, etc.).
​
Workspace setup:
cd ~/Warehouse_Simulation
colcon build
source install/setup.bash
​
PHASE 1 – Mapping:
Terminal commands you already wrote (Gazebo, SLAM, teleop, geotag recorder, RViz), updated to use ~/Warehouse_Simulation paths.
PHASE 2 – Navigation:
Gazebo, localization launch with map + keepout mask under maps/, RViz, Nav2 navigation, waypoint navigator.
