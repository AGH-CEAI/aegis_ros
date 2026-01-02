#include <iostream>
#include <vector>
#include <thread>
#include <string>

#include <grpcpp/grpcpp.h>
#include <rclcpp/rclcpp.hpp>

#include "aegis_grpc/robot_control_service.hpp"
#include "aegis_grpc/robot_read_service.hpp"

std::unique_ptr<grpc::Server>
build_server(const std::string &address,
             std::vector<grpc::Service *> &services) {
  grpc::ServerBuilder builder;
  builder.AddListeningPort(address, grpc::InsecureServerCredentials());

  for (auto *svc : services)
    builder.RegisterService(svc);

  return builder.BuildAndStart();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("grpc_server");

  const std::string address = "0.0.0.0:50051";
  aegis_grpc::RobotReadServiceImpl read_service(node);
  aegis_grpc::RobotControlServiceImpl control_service(node);
  std::vector<grpc::Service *> services = {&read_service, &control_service};

  auto server = build_server(address, services);
  std::cout << "gRPC server listening on " << address << std::endl;

  std::thread grpc_thread([&server]() { server->Wait(); });
  rclcpp::spin(node);

  server->Shutdown();
  grpc_thread.join();
  rclcpp::shutdown();
  return 0;
}
