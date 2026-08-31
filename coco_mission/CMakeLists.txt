cmake_minimum_required(VERSION 3.5)
project(coco_mission)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# mission_states.py is a library, not an entry point, but it installs
# beside the node because that is how `import mission_states` resolves:
# a script run out of lib/<pkg> gets its own directory on sys.path.
install(
  PROGRAMS
    scripts/mission_hud.py
    scripts/mission_executive.py
    scripts/mission_states.py
    # localization_health.py is the same shape as mission_states.py: a
    # pure library beside the nodes so `import localization_health`
    # resolves for anything run out of lib/<pkg>. C2-M5.0 shipped it
    # unwired, by design; C2-M5.1 wired it to localization_monitor.py.
    scripts/localization_health.py
    scripts/localization_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(coco_mission_pytest test
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
