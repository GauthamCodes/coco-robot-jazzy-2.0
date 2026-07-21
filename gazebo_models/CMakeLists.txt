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

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # scripts/ holds operator tools (map_drive.py, verify_sim.py) that were
  # neither tested nor linted because this package had no test block at all.
  ament_add_pytest_test(${PROJECT_NAME}_tests test)
endif()

ament_package()
