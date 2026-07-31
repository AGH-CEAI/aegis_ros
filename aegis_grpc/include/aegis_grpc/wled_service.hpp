#ifndef AEGIS_GRPC__WLED_SERVICE_HPP_
#define AEGIS_GRPC__WLED_SERVICE_HPP_

#include <memory>
#include <grpcpp/grpcpp.h>
#include <rclcpp/rclcpp.hpp>

#include "proto_aegis_grpc/v1/wled_service.grpc.pb.h"

#include <wled_interfaces/srv/change_scene.hpp>
#include <wled_interfaces/srv/define_scene.hpp>
#include <wled_interfaces/srv/get_scenes.hpp>
#include <wled_interfaces/srv/get_sections.hpp>

namespace aegis_grpc {

class WledServiceImpl final : public aegis::grpc::v1::WledService::Service {
 public:
  explicit WledServiceImpl(std::shared_ptr<rclcpp::Node> node);
  virtual ~WledServiceImpl() = default;

  ::grpc::Status ChangeScene(::grpc::ServerContext* context,
                             const ::aegis::grpc::v1::ChangeSceneRequest* request,
                             ::aegis::grpc::v1::GenericStatusResponse* response) override;

  ::grpc::Status DefineScene(::grpc::ServerContext* context,
                             const ::aegis::grpc::v1::DefineSceneRequest* request,
                             ::aegis::grpc::v1::GenericStatusResponse* response) override;

  ::grpc::Status GetScenes(::grpc::ServerContext* context,
                           const ::aegis::grpc::v1::GetScenesRequest* request,
                           ::aegis::grpc::v1::GetScenesResponse* response) override;

  ::grpc::Status GetSections(::grpc::ServerContext* context,
                             const ::aegis::grpc::v1::GetSectionsRequest* request,
                             ::aegis::grpc::v1::GetSectionsResponse* response) override;

 private:
  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Client<wled_interfaces::srv::ChangeScene>::SharedPtr change_scene_client_;
  rclcpp::Client<wled_interfaces::srv::DefineScene>::SharedPtr define_scene_client_;
  rclcpp::Client<wled_interfaces::srv::GetScenes>::SharedPtr get_scenes_client_;
  rclcpp::Client<wled_interfaces::srv::GetSections>::SharedPtr get_sections_client_;
};

}  // namespace aegis_grpc

#endif  // AEGIS_GRPC__WLED_SERVICE_HPP_
