cmake_minimum_required(VERSION 3.8)
project(aegis_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY config
            meshes
            launch
            urdf
            rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
