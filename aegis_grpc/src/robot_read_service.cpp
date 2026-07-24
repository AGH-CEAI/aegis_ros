#include <cassert>
#include "aegis_grpc/robot_read_service.hpp"

namespace aegis_grpc {

RobotReadServiceImpl::RobotReadServiceImpl(std::shared_ptr<rclcpp::Node> node)
    : node_(node), pose_tf_data_(), wrench_data_(), joint_state_data_() {
  // Initialization parameters
  DeclareROSParameter("tcp_frame", std::string("robotiq_hande_end"), "[str] Init; TF2 frame ID of the end-effector.");
  DeclareROSParameter("base_frame", std::string("world"), "[str] Init; TF2 base frame ID for EE pose lookup.");
  DeclareROSParameter("topic_wrench", std::string("/wrench"), "[str] Init; Sub: topic with the F/T data.");
  DeclareROSParameter("topic_joints", std::string("/joint_states"), "[str] Init; Sub: topic with the joint states.");
  DeclareROSParameter("topic_camera_scene", std::string("/cam_scene/rgb/image_rect"),
                      "[str] Init; Sub: Camera scene image topic.");
  DeclareROSParameter("topic_camera_right", std::string("/cam_tool_right/image_color"),
                      "[str] Init; Sub: Camera scene image topic.");
  DeclareROSParameter("topic_camera_left", std::string("/cam_tool_left/image_color"),
                      "[str] Init; Sub:Camera scene image topic.");
  DeclareROSParameter("topics_history_depth", 1, "[bool] Init; The topics messages history (buffer) size.");
  DeclareROSParameter("target_image_width", 64, "[int] Init; Target output image width in pixels.");
  DeclareROSParameter("target_image_height", 64, "[int] Init; Target output image height in pixels.");
  DeclareROSParameter("enable_image_resize", true, "[bool] Init; Enable resizing images before sending over gRPC.");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);

  wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
      node_->get_parameter("topic_wrench").as_string(), node_->get_parameter("topics_history_depth").as_int(),
      std::bind(&RobotReadServiceImpl::WrenchSubCb, this, std::placeholders::_1));
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      node_->get_parameter("topic_joints").as_string(), node_->get_parameter("topics_history_depth").as_int(),
      std::bind(&RobotReadServiceImpl::JointStateSubCb, this, std::placeholders::_1));
  image_scene_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      node_->get_parameter("topic_camera_scene").as_string(), node_->get_parameter("topics_history_depth").as_int(),
      std::bind(&RobotReadServiceImpl::ImageSceneSubCb, this, std::placeholders::_1));
  image_right_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      node_->get_parameter("topic_camera_right").as_string(), node_->get_parameter("topics_history_depth").as_int(),
      std::bind(&RobotReadServiceImpl::ImageRightSubCb, this, std::placeholders::_1));
  image_left_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      node_->get_parameter("topic_camera_left").as_string(), node_->get_parameter("topics_history_depth").as_int(),
      std::bind(&RobotReadServiceImpl::ImageLeftSubCb, this, std::placeholders::_1));

  tcp_frame_ = node_->get_parameter("tcp_frame").as_string();
  base_frame_ = node_->get_parameter("base_frame").as_string();
  target_img_width_ = node_->get_parameter("target_image_width").as_int();
  target_img_height_ = node_->get_parameter("target_image_height").as_int();
  enable_image_resize_ = node_->get_parameter("enable_image_resize").as_bool();
}

template <class T>
void RobotReadServiceImpl::DeclareROSParameter(const std::string& name,
                                               const T& default_val,
                                               const std::string& description) {
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description = description;

  node_->declare_parameter<T>(name, default_val, param_desc);

  const auto p = node_->get_parameter(name);
  RCLCPP_INFO(node_->get_logger(), "> %s := %s", name.c_str(), p.value_to_string().c_str());
}

void RobotReadServiceImpl::WrenchSubCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  wrench_data_ = msg->wrench;
}

void RobotReadServiceImpl::JointStateSubCb(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  joint_state_data_ = *msg;
}

void RobotReadServiceImpl::ImageSceneSubCb(const sensor_msgs::msg::Image::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(image_scene_mutex_);
  image_scene_data_ = *msg;
}

void RobotReadServiceImpl::ImageRightSubCb(const sensor_msgs::msg::Image::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(image_right_mutex_);
  image_right_data_ = *msg;
}

void RobotReadServiceImpl::ImageLeftSubCb(const sensor_msgs::msg::Image::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(image_left_mutex_);
  image_left_data_ = *msg;
}

grpc::Status RobotReadServiceImpl::GetTCPPose(grpc::ServerContext* context,
                                              const google::protobuf::Empty* request,
                                              proto_aegis_grpc::v1::Pose* response) {
  (void)context;
  (void)request;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    PoseTransformUpdate();
    FillProtoPose(pose_tf_data_, response);
  }
  return grpc::Status::OK;
}

grpc::Status RobotReadServiceImpl::GetWrench(grpc::ServerContext* context,
                                             const google::protobuf::Empty* request,
                                             proto_aegis_grpc::v1::Wrench* response) {
  (void)context;
  (void)request;
  {
    std::lock_guard<std::mutex> lock(wrench_mutex_);
    FillProtoWrench(wrench_data_, response);
  }
  return grpc::Status::OK;
}

grpc::Status RobotReadServiceImpl::GetJointStates(grpc::ServerContext* context,
                                                  const google::protobuf::Empty* request,
                                                  proto_aegis_grpc::v1::JointState* response) {
  (void)context;
  (void)request;
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    FillProtoJointState(joint_state_data_, response);
  }
  return grpc::Status::OK;
}

grpc::Status RobotReadServiceImpl::GetCameraSceneImage(grpc::ServerContext* context,
                                                       const google::protobuf::Empty* request,
                                                       proto_aegis_grpc::v1::Image* response) {
  (void)context;
  (void)request;
  {
    std::lock_guard<std::mutex> lock(image_scene_mutex_);
    FillProtoImage(image_scene_data_, response, scene_resized_);
  }
  return grpc::Status::OK;
}

grpc::Status RobotReadServiceImpl::GetCameraRightImage(grpc::ServerContext* context,
                                                       const google::protobuf::Empty* request,
                                                       proto_aegis_grpc::v1::Image* response) {
  (void)context;
  (void)request;
  {
    std::lock_guard<std::mutex> lock(image_right_mutex_);
    FillProtoImage(image_right_data_, response, right_resized_);
  }
  return grpc::Status::OK;
}

grpc::Status RobotReadServiceImpl::GetCameraLeftImage(grpc::ServerContext* context,
                                                      const google::protobuf::Empty* request,
                                                      proto_aegis_grpc::v1::Image* response) {
  (void)context;
  (void)request;
  {
    std::lock_guard<std::mutex> lock(image_left_mutex_);
    FillProtoImage(image_left_data_, response, left_resized_);
  }
  return grpc::Status::OK;
}

grpc::Status RobotReadServiceImpl::GetRobotState(grpc::ServerContext* context,
                                                 const google::protobuf::Empty* request,
                                                 proto_aegis_grpc::v1::RobotState* response) {
  // TODO(issue#85): Guard the freshness of the data in gRPC server
  (void)context;
  (void)request;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    PoseTransformUpdate();
    FillProtoPose(pose_tf_data_, response->mutable_pose());
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

grpc::Status RobotReadServiceImpl::GetRobotVision(grpc::ServerContext* context,
                                                  const google::protobuf::Empty* request,
                                                  proto_aegis_grpc::v1::RobotVision* response) {
  // TODO(issue#85): Guard the freshness of the data in gRPC server
  (void)context;
  (void)request;
  {
    std::lock_guard<std::mutex> lock(image_scene_mutex_);
    FillProtoImage(image_scene_data_, response->mutable_image_scene(), scene_resized_);
  }
  {
    std::lock_guard<std::mutex> lock(image_right_mutex_);
    FillProtoImage(image_right_data_, response->mutable_image_right(), right_resized_);
  }
  {
    std::lock_guard<std::mutex> lock(image_left_mutex_);
    FillProtoImage(image_left_data_, response->mutable_image_left(), left_resized_);
  }
  return grpc::Status::OK;
}

grpc::Status RobotReadServiceImpl::GetAll(grpc::ServerContext* context,
                                          const google::protobuf::Empty* request,
                                          proto_aegis_grpc::v1::RobotObservation* response) {
  (void)context;
  (void)request;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    PoseTransformUpdate();
    FillProtoPose(pose_tf_data_, response->mutable_robot_state()->mutable_pose());
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
    FillProtoImage(image_scene_data_, response->mutable_robot_vision()->mutable_image_scene(), scene_resized_);
  }
  {
    std::lock_guard<std::mutex> lock(image_right_mutex_);
    FillProtoImage(image_right_data_, response->mutable_robot_vision()->mutable_image_right(), right_resized_);
  }
  {
    std::lock_guard<std::mutex> lock(image_left_mutex_);
    FillProtoImage(image_left_data_, response->mutable_robot_vision()->mutable_image_left(), left_resized_);
  }
  return grpc::Status::OK;
}

void RobotReadServiceImpl::PoseTransformUpdate() {
  try {
    pose_tf_data_ = tf_buffer_->lookupTransform(base_frame_, tcp_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Could not lookup transform from %s to %s: %s",
                         base_frame_.c_str(), tcp_frame_.c_str(), ex.what());
  }
}

void RobotReadServiceImpl::FillProtoPose(const geometry_msgs::msg::TransformStamped& ros,
                                         proto_aegis_grpc::v1::Pose* out) {
  auto* pos = out->mutable_position();
  pos->set_x(ros.transform.translation.x);
  pos->set_y(ros.transform.translation.y);
  pos->set_z(ros.transform.translation.z);

  auto* ori = out->mutable_orientation();
  ori->set_x(ros.transform.rotation.x);
  ori->set_y(ros.transform.rotation.y);
  ori->set_z(ros.transform.rotation.z);
  ori->set_w(ros.transform.rotation.w);
}

void RobotReadServiceImpl::FillProtoWrench(const geometry_msgs::msg::Wrench& ros, proto_aegis_grpc::v1::Wrench* out) {
  auto* f = out->mutable_force();
  f->set_x(ros.force.x);
  f->set_y(ros.force.y);
  f->set_z(ros.force.z);

  auto* t = out->mutable_torque();
  t->set_x(ros.torque.x);
  t->set_y(ros.torque.y);
  t->set_z(ros.torque.z);
}

void RobotReadServiceImpl::FillProtoJointState(const sensor_msgs::msg::JointState& ros,
                                               proto_aegis_grpc::v1::JointState* out) {
  out->clear_name();
  out->clear_position();
  out->clear_velocity();
  out->clear_effort();

  const bool has_velocity = ros.velocity.size() == ros.name.size();
  const bool has_effort = ros.effort.size() == ros.name.size();

  out->mutable_name()->Reserve(AEGIS_JOINT_ORDER.size());
  out->mutable_position()->Reserve(AEGIS_JOINT_ORDER.size());
  if (has_velocity)
    out->mutable_velocity()->Reserve(AEGIS_JOINT_ORDER.size());
  if (has_effort)
    out->mutable_effort()->Reserve(AEGIS_JOINT_ORDER.size());

  for (const auto& target_name : AEGIS_JOINT_ORDER) {
    const int idx = FindJointIndex(ros.name, target_name);
    if (idx < 0) {
      RCLCPP_ERROR(node_->get_logger(), "JointState is missing expected joint: %s", std::string(target_name).c_str());
      continue;
    }

    out->add_name(std::string(target_name));
    out->add_position(ros.position[idx]);
    if (has_velocity)
      out->add_velocity(ros.velocity[idx]);
    if (has_effort)
      out->add_effort(ros.effort[idx]);
  }
}

void RobotReadServiceImpl::FillProtoImage(const sensor_msgs::msg::Image& ros,
                                          proto_aegis_grpc::v1::Image* out,
                                          cv::Mat& buffer) {
  if (ros.encoding != "bgr8") {
    RCLCPP_ERROR(node_->get_logger(), "[CAMERA FRAME: %s] Unsupported image encoding (expected 'bgr8'), got: '%s'",
                 ros.header.frame_id.c_str(), ros.encoding.c_str());
    return;
  }
  if (!enable_image_resize_) {
    out->set_height(ros.height);
    out->set_width(ros.width);
    out->set_step(ros.step);
    out->set_data(const_cast<uint8_t*>(ros.data.data()), static_cast<size_t>(ros.height * ros.step));
    return;
  }

  const cv::Mat src(ros.height, ros.width, CV_8UC3, const_cast<uint8_t*>(ros.data.data()), ros.step);

  cv::resize(src, buffer, cv::Size(target_img_width_, target_img_height_), 0, 0, cv::INTER_LINEAR);

  out->set_height(buffer.rows);
  out->set_width(buffer.cols);
  out->set_step(buffer.step);
  out->set_data(buffer.data, buffer.total() * buffer.elemSize());
}

}  // namespace aegis_grpc
