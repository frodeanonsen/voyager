cmake_minimum_required(VERSION 3.8)
project(robot_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate ROS 2 message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Emotion.msg"
)

# Export dependencies for runtime
ament_export_dependencies(rosidl_default_runtime)

# Install URDF directory
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install launch directory
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install RViz configuration
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Finalize package
ament_package()

