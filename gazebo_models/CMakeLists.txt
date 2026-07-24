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
    scripts/climb_check.py
    scripts/gen_ramp.py
    scripts/map_drive.py
    scripts/verify_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # scripts/ holds operator tools (map_drive.py, verify_sim.py) that were
  # neither tested nor linted because this package had no test block at all.
  # Fast, simulator-free unit tests only — test_integration/ is kept out of
  # this directory so pytest cannot collect the launch test into it.
  ament_add_pytest_test(${PROJECT_NAME}_tests test)

  # Simulator-in-the-loop integration test. Off by default: it starts
  # Gazebo and takes ~2 min, which does not belong in the unit run.
  #   colcon build --packages-select gazebo_models \
  #       --cmake-args -DBUILD_SIM_INTEGRATION_TESTS=ON
  option(BUILD_SIM_INTEGRATION_TESTS
         "Run the Gazebo-in-the-loop bringup test (slow)" OFF)
  if(BUILD_SIM_INTEGRATION_TESTS)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test_integration/test_sim_bringup.launch.py TIMEOUT 300)
  endif()
endif()

ament_package()
