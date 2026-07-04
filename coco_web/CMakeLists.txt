cmake_minimum_required(VERSION 3.5)
project(coco_web)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY web launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
