#include <iostream>
#include <memory>
#include <vector>
#include <thread>
#include <string>

#include <grpcpp/grpcpp.h>
#include <rclcpp/rclcpp.hpp>
#include <grpcpp/ext/proto_server_reflection_plugin.h>

#include "aegis_grpc/robot_control_service.hpp"
#include "aegis_grpc/robot_read_service.hpp"

std::unique_ptr<grpc::Server>
build_server(const std::string &address,
             std::vector<grpc::Service *> &services) {

  grpc::ServerBuilder builder;
  grpc::reflection::InitProtoReflectionServerBuilderPlugin();
  grpc::EnableDefaultHealthCheckService(true);
  builder.AddListeningPort(address, grpc::InsecureServerCredentials());

  for (auto *srv : services)
    builder.RegisterService(srv);

  return builder.BuildAndStart();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("grpc_server");

  //TODO parametrize port
  const std::string address = "0.0.0.0:50051";

  std::cout << "Creating RobotReadService" << std::endl;
  aegis_grpc::RobotReadServiceImpl read_service(node);

  std::cout << "Creating RobotControlService" << std::endl;
  aegis_grpc::RobotControlServiceImpl control_service(node);

  std::cout << "Setuping gRPC server with services" << std::endl;
  std::vector<grpc::Service *> services = {&read_service, &control_service};
  auto server = build_server(address, services);
  std::thread grpc_thread([&server]() { server->Wait(); });
  std::cout << "gRPC server listening on " << address << std::endl;

  rclcpp::spin(node);

  std::cout << "Shutting down the gRPC server" << std::endl;
  server->Shutdown();
  grpc_thread.join();
  rclcpp::shutdown();
  return 0;
}
