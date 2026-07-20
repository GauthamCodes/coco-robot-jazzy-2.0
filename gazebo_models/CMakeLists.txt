cmake_minimum_required(VERSION 3.5)
project(gazebo_models)

find_package(ament_cmake REQUIRED)

# Install launch files, URDF models, mesh assets, and RViz configs
install(
  DIRECTORY launch urdf meshes worlds rviz config maps
  DESTINATION share/${PROJECT_NAME}
)

# Operator scripts, so they are reachable as `ros2 run gazebo_models <script>`
# rather than only by source path from a hardcoded workspace location.
install(
  PROGRAMS
    scripts/map_drive.py
    scripts/verify_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
