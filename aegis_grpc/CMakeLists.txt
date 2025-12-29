# cmake-lint: disable=C0307
cmake_minimum_required(VERSION 3.8)
project(aegis_grpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 is required by gRPC
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------
# Dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_program(GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin REQUIRED)

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
set(PROTO_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto_grpc_aegis/v1")
file(GLOB PROTO_SRC "${PROTO_SRC_DIR}/*.proto")
message(STATUS "Proto files path: ${PROTO_SRC_DIR}")
message(STATUS "Found PROTO_SRC files: ${PROTO_SRC}")

# Specify output directory for header, source, and script files
set(GENERATED_PROTO_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
file(MAKE_DIRECTORY ${GENERATED_PROTO_CPP_DIR})

# These will be in CMAKE_CURRENT_BINARY_DIR by default
include_directories(${GENERATED_PROTO_CPP_DIR} ${CMAKE_CURRENT_BINARY_DIR})

# gRPC C++/Python stubs
set(GRPC_SRCS)
set(GRPC_HDRS)

# Generate C++ and Python code from .proto files
foreach(proto_file ${PROTO_SRC})
  get_filename_component(proto_name ${proto_file} NAME_WE)
  message(STATUS "Processing Proto definition: ${proto_name}")

  # Add custom command to generate C++ code for each .proto file
  set(PB_CC ${GENERATED_PROTO_CPP_DIR}/${proto_name}.pb.cc)
  set(PB_H ${GENERATED_PROTO_CPP_DIR}/${proto_name}.pb.h)
  set(GRPC_CC ${GENERATED_PROTO_CPP_DIR}/${proto_name}.grpc.pb.cc)
  set(GRPC_H ${GENERATED_PROTO_CPP_DIR}/${proto_name}.grpc.pb.h)

  add_custom_command(
    OUTPUT ${PB_CC} ${PB_H} ${GRPC_CC} ${GRPC_H}
    COMMAND
      ${Protobuf_PROTOC_EXECUTABLE} --proto_path=${CMAKE_CURRENT_SOURCE_DIR}
      --grpc_out=${GENERATED_PROTO_CPP_DIR} --cpp_out=${GENERATED_PROTO_CPP_DIR}
      --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN_EXECUTABLE} ${proto_file}
    DEPENDS ${proto_file} ${GRPC_CPP_PLUGIN_EXECUTABLE}
    COMMENT "Compiling ${proto_file} to C++ header & source")
  list(APPEND GRPC_SRCS ${GRPC_CC})
  list(APPEND GRPC_HDRS ${GRPC_H})
endforeach()

# Create a custom target to trigger code generation for all proto files
add_custom_target(
  generate_protos ALL
  DEPENDS ${GRPC_SRCS} ${GRPC_HDRS}
  COMMENT "Generating gRPC and Protobuf headers")

# ------------------------------
# Executable targets

add_executable(grpc_server src/grpc_server.cpp)
target_include_directories(grpc_server PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                              $<INSTALL_INTERFACE:include>)
target_compile_features(grpc_server PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17

install(TARGETS grpc_server DESTINATION lib/${PROJECT_NAME})

ament_package()
