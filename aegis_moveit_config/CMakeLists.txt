cmake_minimum_required(VERSION 3.22)
project(aegis_moveit_config)

find_package(ament_cmake REQUIRED)


# find_package(moveit_common REQUIRED)
# moveit_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  geometry_msgs
  moveit_msgs
  sensor_msgs
  std_srvs
  rclcpp
  rclcpp_lifecycle
  rcpputils
)

foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

add_executable(servo_keyboard_input src/servo_keyboard_input.cpp)
target_include_directories(servo_keyboard_input PUBLIC include)
ament_target_dependencies(servo_keyboard_input  ${THIS_PACKAGE_INCLUDE_DEPENDS})

# # Add executable for using a controller
# add_library(servo_controller_input SHARED src/joystick_servo_example.cpp)
# ament_target_dependencies(servo_controller_input PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# rclcpp_components_register_nodes(servo_controller_input "moveit_servo::JoyToServoPub")

install(
  TARGETS
    servo_keyboard_input
    # servo_controller_input
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
