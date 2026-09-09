cmake_minimum_required(VERSION 3.8)
project(ms616)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY urdf launch rviz meshes config worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install the frontier_explorer Python node as an executable
install(PROGRAMS
  ms616/frontier_explorer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME frontier_explorer
)

ament_package()
