cmake_minimum_required(VERSION 3.8)
project(ms616)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY urdf launch rviz meshes config worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install the auto_controller Python nodes as executables so
# `ros2 run ms616 <name>` and Node(package="ms616", executable="<name>")
# in launch files can find them.
install(PROGRAMS
  auto_controller/autonomous_mapper.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME autonomous_mapper
)

install(PROGRAMS
  auto_controller/obstacle_avoidance.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME obstacle_avoidance
)

install(PROGRAMS
  auto_controller/object_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME object_detector
)

<<<<<<< HEAD
install(PROGRAMS
  auto_controller/dynamic_obstacle_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dynamic_obstacle_manager
)

install(PROGRAMS
  auto_controller/dynamic_obstacle_tracker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dynamic_obstacle_tracker
)

ament_package()
=======
ament_package()
>>>>>>> origin/main
