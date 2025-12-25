cmake_minimum_required(VERSION 3.22)
project(scuttle_description_ros2)

find_package(ament_cmake REQUIRED)

# Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install mesh files
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
