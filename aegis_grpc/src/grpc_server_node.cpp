#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <grpcpp/grpcpp.h>
#include <rclcpp/rclcpp.hpp>

std::unique_ptr<grpc::Server> build_server(
  const std::string& address,
  std::vector<grpc::Service*> services) {
    grpc::ServerBuilder builder;
    builder.AddListeningPort(address, grpc::InsecureServerCredentials());

    for (auto* svc : services) builder.RegisterService(svc);

    return builder.BuildAndStart();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("grpc_server");

    // The ownership of the services is not taken from these variables
    // Float32ServiceImpl float32_service(node);
    // Int32ServiceImpl   int32_service(node);

    const std::string address = "0.0.0.0:50051";
    auto server = build_server(address, {}
        // { &float32_service, &int32_service }
    );
    std::cout << "gRPC server listening on " << address << std::endl;

    std::thread grpc_thread([&server]() {
        server->Wait();
    });

    rclcpp::spin(node);

    server->Shutdown();
    grpc_thread.join();
    rclcpp::shutdown();
    return 0;
}
