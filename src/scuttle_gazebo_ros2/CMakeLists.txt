cmake_minimum_required(VERSION 3.22)
project(scuttle_gazebo_ros2)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install model files
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
