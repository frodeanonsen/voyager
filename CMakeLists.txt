cmake_minimum_required(VERSION 3.8)
project(robot_control)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install the URDF directory
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install the launch directory (if you have launch files)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install RViz configuration (optional, if you have RViz files)
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Emotion.msg"
)

ament_package()

