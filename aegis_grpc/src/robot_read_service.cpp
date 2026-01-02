#include "aegis_grpc/robot_read_service.hpp"

namespace aegis_grpc {

RobotReadServiceImpl::RobotReadServiceImpl(std::shared_ptr<rclcpp::Node> node)
    : node_(node), pose_data_(), wrench_data_(), joint_state_data_() {
      //TODO parametrize topic names from ros arguments
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::Pose>(
      "/tcp_pose", 10,
      std::bind(&RobotReadServiceImpl::PoseSubCb, this,
                std::placeholders::_1));
  wrench_sub_ = node_->create_subscription<geometry_msgs::msg::Wrench>(
      "/wrench", 10,
      std::bind(&RobotReadServiceImpl::WrenchSubCb, this,
                std::placeholders::_1));
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&RobotReadServiceImpl::JointStateSubCb, this,
                std::placeholders::_1));
}

void RobotReadServiceImpl::PoseSubCb(
    const geometry_msgs::msg::Pose::SharedPtr msg) {
  pose_data_ = *msg;
}

void RobotReadServiceImpl::WrenchSubCb(
    const geometry_msgs::msg::Wrench::SharedPtr msg) {
  wrench_data_ = *msg;
}

void RobotReadServiceImpl::JointStateSubCb(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  joint_state_data_ = *msg;
}

grpc::Status
RobotReadServiceImpl::GetTCPPose(grpc::ServerContext *context,
                                 const google::protobuf::Empty *request,
                                 proto_grpc_aegis::v1::Pose *response) {
    (void) context;
    (void) request;

    FillProtoPose(pose_data_, response);
    return grpc::Status::OK;
}

grpc::Status
RobotReadServiceImpl::GetWrench(grpc::ServerContext *context,
                                const google::protobuf::Empty *request,
                                proto_grpc_aegis::v1::Wrench *response) {
  (void) context;
  (void) request;
  FillProtoWrench(wrench_data_, response);
  return grpc::Status::OK;
}

grpc::Status RobotReadServiceImpl::GetJointState(
    grpc::ServerContext *context, const google::protobuf::Empty *request,
    proto_grpc_aegis::v1::JointState *response) {
  (void) context;
  (void) request;
  FillProtoJointState(joint_state_data_, response);
  return grpc::Status::OK;
}

grpc::Status
RobotReadServiceImpl::GetAll(grpc::ServerContext *context,
                             const google::protobuf::Empty *request,
                             proto_grpc_aegis::v1::RobotState *response) {
  (void) context;
  (void) request;
  FillProtoPose(pose_data_, response->mutable_pose());
  FillProtoWrench(wrench_data_, response->mutable_wrench());
  FillProtoJointState(joint_state_data_, response->mutable_joint_state());
  return grpc::Status::OK;
}

void RobotReadServiceImpl::FillProtoPose(
    const geometry_msgs::msg::Pose& ros,
    proto_grpc_aegis::v1::Pose* out) {
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
    proto_grpc_aegis::v1::Wrench* out) {
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
    proto_grpc_aegis::v1::JointState* out) {
  out->clear_name();
  out->clear_position();
  out->clear_velocity();
  out->clear_effort();

  for (const auto& v : ros.name) out->add_name(v);
  for (const auto& v : ros.position) out->add_position(v);
  for (const auto& v : ros.velocity) out->add_velocity(v);
  for (const auto& v : ros.effort) out->add_effort(v);
}


} // namespace aegis_grpc
