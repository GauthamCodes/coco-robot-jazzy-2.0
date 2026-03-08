cmake_minimum_required(VERSION 3.5)
project(gazebo_models)

find_package(ament_cmake REQUIRED)

# Install launch files, URDF models and mesh assets
install(
  DIRECTORY launch urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
