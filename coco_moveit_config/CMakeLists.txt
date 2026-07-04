cmake_minimum_required(VERSION 3.5)
project(coco_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/pick_place.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
