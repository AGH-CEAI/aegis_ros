#include "aegis_grpc/robot_read_service.hpp"

namespace aegis_grpc {

RobotReadServiceImpl::RobotReadServiceImpl(std::shared_ptr<rclcpp::Node> node)
    : node_(node), pose_data_(), wrench_data_(), joint_state_data_() {

  // Initialization parameters
  DeclareROSParameter("topic_pose", "/tcp_pose", "[str] Init; Sub: topic with the TCP pose data.");
  DeclareROSParameter("topic_wrench", "/wrench", "[str] Init; Sub: topic with the F/T data.");
  DeclareROSParameter("topic_joints", "/joint_states", "[str] Init; Sub: topic with the joint states.");
  DeclareROSParameter("topic_camera_scene", "/cam_scene/rgb/image_rect", "[str] Camera scene image topic");
  DeclareROSParameter("topic_camera_right", "/cam_tool_right/image_raw", "[str] Camera scene image topic");
  DeclareROSParameter("topic_camera_left", "/cam_tool_left/image_raw", "[str] Camera scene image topic");


  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      node_->get_parameter("topic_pose").as_string(), 10,
      std::bind(&RobotReadServiceImpl::PoseSubCb, this,
                std::placeholders::_1));
  wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
      node_->get_parameter("topic_wrench").as_string(), 10,
      std::bind(&RobotReadServiceImpl::WrenchSubCb, this,
                std::placeholders::_1));
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      node_->get_parameter("topic_joints").as_string(), 10,
      std::bind(&RobotReadServiceImpl::JointStateSubCb, this,
                std::placeholders::_1));
  image_scene_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      node_->get_parameter("topic_camera_scene").as_string(), 10,
      std::bind(&RobotReadServiceImpl::ImageSceneSubCb, this,
                std::placeholders::_1));
  image_right_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      node_->get_parameter("topic_camera_right").as_string(), 10,
      std::bind(&RobotReadServiceImpl::ImageRightSubCb, this,
                std::placeholders::_1));
  image_left_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      node_->get_parameter("topic_camera_left").as_string(), 10,
      std::bind(&RobotReadServiceImpl::ImageLeftSubCb, this,
                std::placeholders::_1));
}

void RobotReadServiceImpl::DeclareROSParameter(const std::string& name, const std::string& default_val, const std::string& description) {
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description = description;
  node_->declare_parameter(name, default_val, param_desc);

  const auto p = node_->get_parameter(name);
  RCLCPP_INFO(node_->get_logger(), "> %s := %s",
              name.c_str(),
              p.value_to_string().c_str());
}

void RobotReadServiceImpl::PoseSubCb(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(pose_mutex_);
  pose_data_ = msg->pose;
}

void RobotReadServiceImpl::WrenchSubCb(
    const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  wrench_data_ = msg->wrench;
}

void RobotReadServiceImpl::JointStateSubCb(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  joint_state_data_ = *msg;
}

void RobotReadServiceImpl::ImageSceneSubCb(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(image_scene_mutex_);
  image_scene_data_ = *msg;
}

void RobotReadServiceImpl::ImageRightSubCb(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(image_right_mutex_);
  image_right_data_ = *msg;
}

void RobotReadServiceImpl::ImageLeftSubCb(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(image_left_mutex_);
  image_left_data_ = *msg;
}

grpc::Status
RobotReadServiceImpl::GetTCPPose(grpc::ServerContext *context,
                                 const google::protobuf::Empty *request,
                                 proto_aegis_grpc::v1::Pose *response) {
    (void) context;
    (void) request;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      FillProtoPose(pose_data_, response);
    }
    return grpc::Status::OK;
}

grpc::Status
RobotReadServiceImpl::GetWrench(grpc::ServerContext *context,
                                const google::protobuf::Empty *request,
                                proto_aegis_grpc::v1::Wrench *response) {
  (void) context;
  (void) request;
  {
    std::lock_guard<std::mutex> lock(wrench_mutex_);
    FillProtoWrench(wrench_data_, response);
  }
  return grpc::Status::OK;
}

grpc::Status RobotReadServiceImpl::GetJointStates(
    grpc::ServerContext *context, const google::protobuf::Empty *request,
    proto_aegis_grpc::v1::JointState *response) {
  (void) context;
  (void) request;
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    FillProtoJointState(joint_state_data_, response);
  }
  return grpc::Status::OK;
}

grpc::Status RobotReadServiceImpl::GetCameraSceneImage(
    grpc::ServerContext* context,
    const google::protobuf::Empty* request,
    proto_aegis_grpc::v1::Image* response) {
  (void)context;
  (void)request;
  {
    std::lock_guard<std::mutex> lock(image_scene_mutex_);
    FillProtoImage(image_scene_data_, response);
  }
  return grpc::Status::OK;
}

grpc::Status RobotReadServiceImpl::GetCameraRightImage(
    grpc::ServerContext* context,
    const google::protobuf::Empty* request,
    proto_aegis_grpc::v1::Image* response) {
  (void)context;
  (void)request;
  {
    std::lock_guard<std::mutex> lock(image_right_mutex_);
    FillProtoImage(image_right_data_, response);
  }
  return grpc::Status::OK;
}

grpc::Status RobotReadServiceImpl::GetCameraLeftImage(
    grpc::ServerContext* context,
    const google::protobuf::Empty* request,
    proto_aegis_grpc::v1::Image* response) {
  (void)context;
  (void)request;
  {
    std::lock_guard<std::mutex> lock(image_left_mutex_);
    FillProtoImage(image_left_data_, response);
  }
  return grpc::Status::OK;
}

grpc::Status
RobotReadServiceImpl::GetAll(grpc::ServerContext *context,
                             const google::protobuf::Empty *request,
                             proto_aegis_grpc::v1::RobotState *response) {
  (void) context;
  (void) request;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    FillProtoPose(pose_data_, response->mutable_pose());
  }
  {
    std::lock_guard<std::mutex> lock(wrench_mutex_);
    FillProtoWrench(wrench_data_, response->mutable_wrench());
  }
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    FillProtoJointState(joint_state_data_, response->mutable_joint_state());
  }
  return grpc::Status::OK;
}

grpc::Status RobotReadServiceImpl::GetAllVis(
    grpc::ServerContext* context,
    const google::protobuf::Empty* request,
    proto_aegis_grpc::v1::RobotStateVis* response) {

    (void)context;
    (void)request;

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        FillProtoPose(pose_data_, response->mutable_robot_state()->mutable_pose());
    }
    {
        std::lock_guard<std::mutex> lock(wrench_mutex_);
        FillProtoWrench(wrench_data_, response->mutable_robot_state()->mutable_wrench());
    }
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        FillProtoJointState(joint_state_data_, response->mutable_robot_state()->mutable_joint_state());
    }
    {
        std::lock_guard<std::mutex> lock(image_scene_mutex_);
        FillProtoImage(image_scene_data_, response->mutable_image_scene());
    }
    {
        std::lock_guard<std::mutex> lock(image_right_mutex_);
        FillProtoImage(image_right_data_, response->mutable_image_right());
    }
    {
        std::lock_guard<std::mutex> lock(image_left_mutex_);
        FillProtoImage(image_left_data_, response->mutable_image_left());
    }

    return grpc::Status::OK;
}

void RobotReadServiceImpl::FillProtoPose(
    const geometry_msgs::msg::Pose& ros,
    proto_aegis_grpc::v1::Pose* out) {
  auto* pos = out->mutable_position();
  pos->set_x(ros.position.x);
  pos->set_y(ros.position.y);
  pos->set_z(ros.position.z);

  auto* ori = out->mutable_orientation();
  ori->set_x(ros.orientation.x);
  ori->set_y(ros.orientation.y);
  ori->set_z(ros.orientation.z);
  ori->set_w(ros.orientation.w);
}

void RobotReadServiceImpl::FillProtoWrench(
    const geometry_msgs::msg::Wrench& ros,
    proto_aegis_grpc::v1::Wrench* out) {
  auto* f = out->mutable_force();
  f->set_x(ros.force.x);
  f->set_y(ros.force.y);
  f->set_z(ros.force.z);

  auto* t = out->mutable_torque();
  t->set_x(ros.torque.x);
  t->set_y(ros.torque.y);
  t->set_z(ros.torque.z);
}

void RobotReadServiceImpl::FillProtoJointState(
    const sensor_msgs::msg::JointState& ros,
    proto_aegis_grpc::v1::JointState* out) {
  out->clear_name();
  out->clear_position();
  out->clear_velocity();
  out->clear_effort();

  for (const auto& v : ros.name) out->add_name(v);
  for (const auto& v : ros.position) out->add_position(v);
  for (const auto& v : ros.velocity) out->add_velocity(v);
  for (const auto& v : ros.effort) out->add_effort(v);
}

void RobotReadServiceImpl::FillProtoImage(
    const sensor_msgs::msg::Image& ros,
    proto_aegis_grpc::v1::Image* out) {
  out->set_height(ros.height);
  out->set_width(ros.width);

  if (ros.encoding == "bgr8") {
      out->set_encoding(proto_aegis_grpc::v1::Image::BGR8);
  } else if (ros.encoding == "bayer_rggb8") {
      out->set_encoding(proto_aegis_grpc::v1::Image::BAYER_RGGB8);
  } else {
      out->set_encoding(proto_aegis_grpc::v1::Image::UNKNOWN);
  }

  out->set_step(ros.step);
  out->set_data(ros.data.data(), ros.data.size());
}


} // namespace aegis_grpc
