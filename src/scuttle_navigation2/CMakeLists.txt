cmake_minimum_required(VERSION 3.22)
project(scuttle_navigation2)

find_package(ament_cmake REQUIRED)

# Install launch files and config
install(DIRECTORY 
  launch 
  config 
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/obstacle_speed_controller.py
  scripts/emergency_stop.py
  scripts/wifi_monitor.py
  scripts/vertical_scanner.py
  scripts/qr_code_scanner.py
  scripts/competition_waypoint_navigator.py
  scripts/competition_server.py
  scripts/waypoint_validator.py
  scripts/interactive_waypoint_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
