cmake_minimum_required(VERSION 3.5)
project(coco_mission)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
