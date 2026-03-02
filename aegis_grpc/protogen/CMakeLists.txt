# cmake-lint: disable=C0307
cmake_minimum_required(VERSION 3.8)
project(proto_generator)

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
set(MAIN_CMAKE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(PROTO_SRC_DIR "${MAIN_CMAKE_DIR}/proto_aegis_grpc/v1")
file(GLOB PROTO_SRC "${PROTO_SRC_DIR}/*.proto")
message(STATUS "Proto files path: ${PROTO_SRC_DIR}")
message(STATUS "Found PROTO_SRC files: ${PROTO_SRC}")

# Specify output directory for header, and source (CPP) files
set(GENERATED_PROTO_CPP_DIR_ROOT "${MAIN_CMAKE_DIR}/include")
set(GENERATED_PROTO_CPP_DIR "${GENERATED_PROTO_CPP_DIR_ROOT}/proto_aegis_grpc/v1")
file(MAKE_DIRECTORY ${GENERATED_PROTO_CPP_DIR})

# Specify output directory for script (Python) files
set(GENERATED_PROTO_PYTHON_DIR_ROOT "${MAIN_CMAKE_DIR}/python_proto")
set(GENERATED_PROTO_PYTHON_DIR "${GENERATED_PROTO_PYTHON_DIR_ROOT}/proto_aegis_grpc/v1")
file(MAKE_DIRECTORY ${GENERATED_PROTO_PYTHON_DIR})

# gRPC C++/Python stubs
set(PROTO_SRCS)
set(PROTO_HDRS)
set(PROTO_PYS)
set(GRPC_SRCS)
set(GRPC_HDRS)
set(GRPC_PYS)
# add_library(proto_objects OBJECT)

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
      ${Protobuf_PROTOC_EXECUTABLE} --proto_path=${MAIN_CMAKE_DIR}
      --grpc_out=${GENERATED_PROTO_CPP_DIR_ROOT} --cpp_out=${GENERATED_PROTO_CPP_DIR_ROOT}
      --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN_EXECUTABLE} ${proto_file}
    DEPENDS ${proto_file} ${GRPC_CPP_PLUGIN_EXECUTABLE}
    COMMENT "Compiling ${proto_file} to C++ header & source files")

  list(APPEND PROTO_SRCS ${PB_CC})
  list(APPEND PROTO_HDRS ${PB_H})
  list(APPEND GRPC_SRCS ${GRPC_CC})
  list(APPEND GRPC_HDRS ${GRPC_H})
  # target_sources(proto_objects PRIVATE ${PB_CC} ${GRPC_CC}) set_source_files_properties(${PB_CC}
  # ${GRPC_CC} PROPERTIES GENERATED TRUE)

  # ------------------------------
  # Python

  set(PB_PY ${GENERATED_PROTO_PYTHON_DIR}/${proto_name}_pb2.py)
  set(GRPC_PY ${GENERATED_PROTO_PYTHON_DIR}/${proto_name}_pb2_grpc.py)

  add_custom_command(
    OUTPUT ${PB_PY} ${GRPC_PY}
    COMMAND
      python3 -m grpc_tools.protoc ARGS --proto_path=${MAIN_CMAKE_DIR}
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
  # proto_objects
  COMMENT "Generating gRPC and Protobuf files (C++/Python)")

set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS} ${GRPC_SRCS} ${GRPC_HDRS}
                            PROPERTIES GENERATED TRUE)

# Export variables to parent scope
set(PROTO_SRCS
    ${PROTO_SRCS}
    PARENT_SCOPE)
set(PROTO_HDRS
    ${PROTO_HDRS}
    PARENT_SCOPE)
set(GRPC_SRCS
    ${GRPC_SRCS}
    PARENT_SCOPE)
set(GRPC_HDRS
    ${GRPC_HDRS}
    PARENT_SCOPE)
set(PROTO_PYS
    ${PROTO_PYS}
    PARENT_SCOPE)
set(GRPC_PYS
    ${GRPC_PYS}
    PARENT_SCOPE)
set(GENERATED_PROTO_CPP_DIR
    ${GENERATED_PROTO_CPP_DIR}
    PARENT_SCOPE)
