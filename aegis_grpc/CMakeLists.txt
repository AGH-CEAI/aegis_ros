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
    moveit_ros_planning
    moveit_ros_planning_interface
    sensor_msgs
    std_srvs)

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

# ------------------------------
# Protbuf & gRPC generation

# Set the directory where .proto files are located
set(PROTO_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto_aegis_grpc/v1")
file(GLOB PROTO_SRC "${PROTO_SRC_DIR}/*.proto")
message(STATUS "Proto files path: ${PROTO_SRC_DIR}")
message(STATUS "Found PROTO_SRC files: ${PROTO_SRC}")

# Specify output directory for header, and source (CPP) files
set(GENERATED_PROTO_CPP_DIR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(GENERATED_PROTO_CPP_DIR "${GENERATED_PROTO_CPP_DIR_ROOT}/proto_aegis_grpc/v1")
file(MAKE_DIRECTORY ${GENERATED_PROTO_CPP_DIR})

# Specify output directory for script (Python) files
set(GENERATED_PROTO_PYTHON_DIR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/python_proto")
set(GENERATED_PROTO_PYTHON_DIR "${GENERATED_PROTO_PYTHON_DIR_ROOT}/proto_aegis_grpc/v1")
file(MAKE_DIRECTORY ${GENERATED_PROTO_PYTHON_DIR})

# gRPC C++/Python stubs
set(PROTO_SRCS)
set(PROTO_HDRS)
set(PROTO_PYS)
set(GRPC_SRCS)
set(GRPC_HDRS)
set(GRPC_PYS)

# Generate C++ and Python code from .proto files
foreach(proto_file ${PROTO_SRC})
  get_filename_component(proto_name ${proto_file} NAME_WE)
  message(STATUS "Processing Proto definition: ${proto_name}")

  # ------------------------------
  # C++

  # Add custom command to generate C++ code for each .proto file
  set(PB_CC ${GENERATED_PROTO_CPP_DIR}/${proto_name}.pb.cc)
  set(PB_H ${GENERATED_PROTO_CPP_DIR}/${proto_name}.pb.h)
  set(GRPC_CC ${GENERATED_PROTO_CPP_DIR}/${proto_name}.grpc.pb.cc)
  set(GRPC_H ${GENERATED_PROTO_CPP_DIR}/${proto_name}.grpc.pb.h)

  add_custom_command(
    OUTPUT ${PB_CC} ${PB_H} ${GRPC_CC} ${GRPC_H}
    COMMAND
      # ${Protobuf_PROTOC_EXECUTABLE} --proto_path=${CMAKE_CURRENT_SOURCE_DIR}
      ${Protobuf_PROTOC_EXECUTABLE} --proto_path=${CMAKE_CURRENT_SOURCE_DIR}
      --grpc_out=${GENERATED_PROTO_CPP_DIR_ROOT} --cpp_out=${GENERATED_PROTO_CPP_DIR_ROOT}
      --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN_EXECUTABLE} ${proto_file}
    DEPENDS ${proto_file} ${GRPC_CPP_PLUGIN_EXECUTABLE}
    COMMENT "Compiling ${proto_file} to C++ header & source files")

  list(APPEND PROTO_SRCS ${PB_CC})
  list(APPEND PROTO_HDRS ${PB_H})
  list(APPEND GRPC_SRCS ${GRPC_CC})
  list(APPEND GRPC_HDRS ${GRPC_H})

  # ------------------------------
  # Python

  set(PB_PY ${GENERATED_PROTO_PYTHON_DIR}/${proto_name}_pb2.py)
  set(GRPC_PY ${GENERATED_PROTO_PYTHON_DIR}/${proto_name}_pb2_grpc.py)

  add_custom_command(
    OUTPUT ${PB_PY} ${GRPC_PY}
    COMMAND
      python3 -m grpc_tools.protoc ARGS --proto_path=${CMAKE_CURRENT_SOURCE_DIR}
      --python_out=${GENERATED_PROTO_PYTHON_DIR_ROOT}
      --grpc_python_out=${GENERATED_PROTO_PYTHON_DIR_ROOT} ${proto_file}
    DEPENDS ${proto_file}
    COMMENT "Compiling ${proto_file} to Python script files")
  list(APPEND PROTO_PYS ${PB_PY})
  list(APPEND GRPC_PYS ${GRPC_PY})

endforeach()

# Create a custom target to trigger code generation for all proto files
add_custom_target(
  generate_protos ALL
  DEPENDS ${PROTO_SRCS}
          ${PROTO_HDRS}
          ${PROTO_PYS}
          ${GRPC_SRCS}
          ${GRPC_HDRS}
          ${GRPC_PYS}
  COMMENT "Generating gRPC and Protobuf files (C++/Python)")

# ------------------------------
# Static library with all CPP files

add_library(${PROJECT_NAME}_lib STATIC src/robot_read_service.cpp src/robot_control_service.cpp)

target_include_directories(
  ${PROJECT_NAME}_lib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                             $<INSTALL_INTERFACE:include> ${GENERATED_PROTO_CPP_DIR})

ament_target_dependencies(${PROJECT_NAME}_lib ${ROS_DEPENDS})

target_link_libraries(${PROJECT_NAME}_lib gRPC::grpc++ protobuf::libprotobuf)

add_dependencies(${PROJECT_NAME}_lib generate_protos)
target_sources(${PROJECT_NAME}_lib PRIVATE ${PROTO_SRCS} ${GRPC_SRCS})

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
