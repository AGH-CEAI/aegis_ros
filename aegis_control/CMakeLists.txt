cmake_minimum_required(VERSION 3.8)
project(aegis_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/pylon_roi_setter.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/pylon_white_balance_setter.py DESTINATION lib/${PROJECT_NAME})

ament_package()
