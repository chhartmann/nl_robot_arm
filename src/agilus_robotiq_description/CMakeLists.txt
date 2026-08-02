cmake_minimum_required(VERSION 3.10)
project(agilus_robotiq_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch worlds meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
