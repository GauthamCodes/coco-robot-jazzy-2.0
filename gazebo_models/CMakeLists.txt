cmake_minimum_required(VERSION 3.5)
project(gazebo_models)

find_package(ament_cmake REQUIRED)

# Install launch files, URDF models, mesh assets, and RViz configs
install(
  DIRECTORY launch urdf meshes worlds rviz config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
