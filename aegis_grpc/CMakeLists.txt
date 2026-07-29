# cmake-lint: disable=C0307
cmake_minimum_required(VERSION 3.8)
project(aegis_grpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# C++17 is required by gRPC
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------
# Dependencies

find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)

set(ROS_DEPENDS
    rclcpp
    rclcpp_action
    control_msgs
    controller_manager_msgs
    cv_bridge
    geometry_msgs
    moveit_msgs
    moveit_ros_planning_interface
    sensor_msgs
    std_srvs
    wled_interfaces)

# Hide warnings from importing moveit packages
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

foreach(ros_dep ${ROS_DEPENDS})
  find_package(${ros_dep} REQUIRED)
endforeach()

find_program(GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin REQUIRED)

# gRPC package for Ubuntu 22.04 (ROS Humble) can't be find by CMake's find_package()
# https://bugs.launchpad.net/ubuntu/+source/grpc/+bug/1935709
find_package(gRPC CONFIG QUIET)
if(NOT gRPC_FOUND)
  message(STATUS "gRPC CONFIG not found, using manual library detection")

  # Find gRPC libraries manually
  find_library(GRPC_LIBRARY NAMES grpc++ REQUIRED)
  find_library(GRPC_UNSECURE_LIBRARY NAMES grpc REQUIRED)

  message(STATUS "Found gRPC library: ${GRPC_LIBRARY}")
  message(STATUS "Found gRPC insecure library: ${GRPC_UNSECURE_LIBRARY}")

  # Create imported target for gRPC::grpc++
  if(NOT TARGET gRPC::grpc++)
    add_library(gRPC::grpc++ UNKNOWN IMPORTED)
    set_target_properties(
      gRPC::grpc++ PROPERTIES IMPORTED_LOCATION "${GRPC_LIBRARY}" INTERFACE_LINK_LIBRARIES
                                                                  "${GRPC_UNSECURE_LIBRARY}")
    message(STATUS "Setup the gRPC::grpc++ library")
  endif()
else()
  message(STATUS "Found gRPC via CONFIG")
endif()
find_package(Protobuf REQUIRED)

add_subdirectory(protogen)

# ------------------------------
# Static library with all CPP files
add_library(${PROJECT_NAME}_lib STATIC 
  src/robot_read_service.cpp 
  src/robot_control_service.cpp
  src/wled_service.cpp)
add_dependencies(${PROJECT_NAME}_lib generate_protos)

target_include_directories(
  ${PROJECT_NAME}_lib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                             $<INSTALL_INTERFACE:include> ${GENERATED_PROTO_CPP_DIR})

ament_target_dependencies(${PROJECT_NAME}_lib ${ROS_DEPENDS})

target_link_libraries(${PROJECT_NAME}_lib gRPC::grpc++ protobuf::libprotobuf)

target_sources(${PROJECT_NAME}_lib PRIVATE ${PROTO_SRCS} ${GRPC_SRCS})

# Mark generated sources as generated so CMake doesn't check for them during configuration
set_source_files_properties(${PROTO_SRCS} ${GRPC_SRCS} PROPERTIES GENERATED TRUE)

# ------------------------------
# Executable target

add_executable(grpc_server src/grpc_server_node.cpp)
add_dependencies(grpc_server ${PROJECT_NAME}_lib generate_protos)
target_compile_features(grpc_server PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17

target_link_libraries(
  grpc_server
  gRPC::grpc++
  grpc++_reflection
  protobuf::libprotobuf
  opencv_core
  opencv_imgproc
  ${PROJECT_NAME}_lib)

ament_target_dependencies(grpc_server ${ROS_DEPENDS})

install(TARGETS grpc_server ${PROJECT_NAME}_lib DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
