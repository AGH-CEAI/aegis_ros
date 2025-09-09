cmake_minimum_required(VERSION 3.22)
project(aegis_moveit_config)


find_package(moveit_common REQUIRED)
moveit_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  # control_toolbox
  geometry_msgs
  # moveit_core
  moveit_msgs
  # moveit_ros_planning
  # pluginlib
  # rclcpp
  # rclcpp_components
  # realtime_tools
  sensor_msgs
  # std_msgs
  std_srvs
  # tf2_eigen
  # trajectory_msgs
)

# Add executable for using a controller
add_library(servo_controller_input SHARED src/teleop_demo/joystick_servo_example.cpp)
ament_target_dependencies(servo_controller_input PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(servo_controller_input "moveit_servo::JoyToServoPub")

find_package(ament_cmake REQUIRED)

ament_package()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
