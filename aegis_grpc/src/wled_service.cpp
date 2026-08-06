#include "aegis_grpc/wled_service.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace aegis_grpc {

WledServiceImpl::WledServiceImpl(std::shared_ptr<rclcpp::Node> node) : node_(node) {
  change_scene_client_ = node_->create_client<wled_interfaces::srv::ChangeScene>("wled_change_scene");
  define_scene_client_ = node_->create_client<wled_interfaces::srv::DefineScene>("wled_define_scene");
  get_scenes_client_ = node_->create_client<wled_interfaces::srv::GetScenes>("wled_get_scenes");
  get_sections_client_ = node_->create_client<wled_interfaces::srv::GetSections>("wled_get_sections");

  rclcpp::QoS qos_profile(1);
  qos_profile.transient_local();
  qos_profile.reliable();

  effects_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/wled_effects", qos_profile,
      [this](const std_msgs::msg::String::SharedPtr msg) { this->cached_effects_data_ = msg->data; });
}

::grpc::Status WledServiceImpl::ChangeScene(::grpc::ServerContext* /*context*/,
                                            const ::proto_aegis_grpc::v1::ChangeSceneRequest* request,
                                            ::proto_aegis_grpc::v1::GenericStatusResponse* response) {
  if (!change_scene_client_->wait_for_service(10s)) {
    return ::grpc::Status(::grpc::StatusCode::UNAVAILABLE, "ROS 2 ChangeScene service not available");
  }

  auto ros_req = std::make_shared<wled_interfaces::srv::ChangeScene::Request>();
  ros_req->scene = request->scene();
  ros_req->section = request->section();
  ros_req->effect_id = request->effect_id();
  ros_req->optional_params = request->optional_params();
  // RCLCPP_INFO(node_->get_logger(), "Sending ChangeScene request to ROS 2 service with scene: %s, section: %s,
  // effect_id: %ld", ros_req->scene.c_str(), ros_req->section.c_str(), ros_req->effect_id);
  auto future = change_scene_client_->async_send_request(ros_req);
  if (future.wait_for(10s) == std::future_status::ready) {
    auto ros_res = future.get();
    response->set_success(ros_res->success);
    response->set_message(ros_res->message);
    return ::grpc::Status::OK;
  }

  return ::grpc::Status(::grpc::StatusCode::DEADLINE_EXCEEDED, "Timeout calling ROS 2 ChangeScene service");
}

::grpc::Status WledServiceImpl::DefineScene(::grpc::ServerContext* /*context*/,
                                            const ::proto_aegis_grpc::v1::DefineSceneRequest* request,
                                            ::proto_aegis_grpc::v1::GenericStatusResponse* response) {
  if (!define_scene_client_->wait_for_service(1s)) {
    return ::grpc::Status(::grpc::StatusCode::UNAVAILABLE, "ROS 2 DefineScene service not available");
  }

  auto ros_req = std::make_shared<wled_interfaces::srv::DefineScene::Request>();
  ros_req->scene_name = request->scene_name();
  ros_req->brightness = request->brightness();
  for (int c : request->color()) {
    ros_req->color.push_back(c);
  }

  auto future = define_scene_client_->async_send_request(ros_req);
  if (future.wait_for(3s) == std::future_status::ready) {
    auto ros_res = future.get();
    response->set_success(ros_res->success);
    response->set_message(ros_res->message);
    return ::grpc::Status::OK;
  }

  return ::grpc::Status(::grpc::StatusCode::DEADLINE_EXCEEDED, "Timeout calling ROS 2 DefineScene service");
}

::grpc::Status WledServiceImpl::GetScenes(::grpc::ServerContext* /*context*/,
                                          const ::proto_aegis_grpc::v1::GetScenesRequest* /*request*/,
                                          ::proto_aegis_grpc::v1::GetScenesResponse* response) {
  if (!get_scenes_client_->wait_for_service(1s)) {
    return ::grpc::Status(::grpc::StatusCode::UNAVAILABLE, "ROS 2 GetScenes service not available");
  }

  auto ros_req = std::make_shared<wled_interfaces::srv::GetScenes::Request>();
  auto future = get_scenes_client_->async_send_request(ros_req);

  if (future.wait_for(3s) == std::future_status::ready) {
    auto ros_res = future.get();
    for (const auto& name : ros_res->scene_names)
      response->add_scene_names(name);
    for (int b : ros_res->brightnesses)
      response->add_brightnesses(b);
    for (int r : ros_res->colors_r)
      response->add_colors_r(r);
    for (int g : ros_res->colors_g)
      response->add_colors_g(g);
    for (int b : ros_res->colors_b)
      response->add_colors_b(b);
    return ::grpc::Status::OK;
  }

  return ::grpc::Status(::grpc::StatusCode::DEADLINE_EXCEEDED, "Timeout calling ROS 2 GetScenes service");
}

::grpc::Status WledServiceImpl::GetSections(::grpc::ServerContext* /*context*/,
                                            const ::proto_aegis_grpc::v1::GetSectionsRequest* /*request*/,
                                            ::proto_aegis_grpc::v1::GetSectionsResponse* response) {
  if (!get_sections_client_->wait_for_service(1s)) {
    return ::grpc::Status(::grpc::StatusCode::UNAVAILABLE, "ROS 2 GetSections service not available");
  }

  auto ros_req = std::make_shared<wled_interfaces::srv::GetSections::Request>();
  auto future = get_sections_client_->async_send_request(ros_req);

  if (future.wait_for(3s) == std::future_status::ready) {
    auto ros_res = future.get();
    for (const auto& sec : ros_res->section_names)
      response->add_section_names(sec);
    for (int st : ros_res->starts)
      response->add_starts(st);
    for (int sp : ros_res->stops)
      response->add_stops(sp);
    return ::grpc::Status::OK;
  }

  return ::grpc::Status(::grpc::StatusCode::DEADLINE_EXCEEDED, "Timeout calling ROS 2 GetSections service");
}

::grpc::Status WledServiceImpl::StreamEffects(
    ::grpc::ServerContext* /*context*/,
    const ::proto_aegis_grpc::v1::Empty* /*request*/,
    ::grpc::ServerWriter<::proto_aegis_grpc::v1::WledEffectsResponse>* writer) {
  if (cached_effects_data_.empty()) {
    return ::grpc::Status(::grpc::StatusCode::UNAVAILABLE, "Effects not cached yet from /wled_effects topic");
  }

  ::proto_aegis_grpc::v1::WledEffectsResponse response;
  response.set_effects_json_or_text(cached_effects_data_);

  writer->Write(response);

  return ::grpc::Status::OK;
}

}  // namespace aegis_grpc
