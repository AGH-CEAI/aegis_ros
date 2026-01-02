#include <chrono>
#include "aegis_grpc/robot_control_service.hpp"

using namespace std::chrono_literals;

namespace aegis_grpc {

RobotControlServiceImpl::RobotControlServiceImpl(
    std::shared_ptr<rclcpp::Node> node)
    : node_(node), servo_joint_msg_(), servo_tcp_msg_(),
      gripper_cmd_success_(false), gripper_cmd_msg_("") {
  servo_joint_pub_ = node_->create_publisher<control_msgs::msg::JointJog>(
      "/servo_node/delta_joint_cmds", 10);
  servo_tcp_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);
  gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      node_, "/gripper_controller/gripper_cmd");

  // TODO pass arguments to setup the frequencies
  auto pub_period = 500ms;
  pub_timer_ = node_->create_wall_timer(
      pub_period, std::bind(&RobotControlServiceImpl::PublishLoop, this));
}

void RobotControlServiceImpl::PublishLoop() {
  // TODO add mutexes
  // TODO add mechanism for "frequency ratio"
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp = node_->now();
  twist_msg.header.frame_id = "base_link";
  twist_msg.twist = servo_tcp_msg_;
  servo_tcp_pub_->publish(twist_msg);

  control_msgs::msg::JointJog jog_msg = servo_joint_msg_;
  jog_msg.header.stamp = node_->now();
  servo_joint_pub_->publish(jog_msg);
}

grpc::Status RobotControlServiceImpl::ServoJoint(
    grpc::ServerContext *context, const proto_grpc_aegis::v1::JointJog *request,
    google::protobuf::Empty *response) {

  (void)context;
  (void)response;

  auto ros_msg = control_msgs::msg::JointJog();

  ros_msg.joint_names.reserve(request->joint_names_size());
  for (int i = 0; i < request->joint_names_size(); ++i) {
    ros_msg.joint_names.push_back(request->joint_names(i));
  }

  ros_msg.displacements.reserve(request->displacements_size());
  for (int i = 0; i < request->displacements_size(); ++i) {
    ros_msg.displacements.push_back(request->displacements(i));
  }

  ros_msg.velocities.reserve(request->velocities_size());
  for (int i = 0; i < request->velocities_size(); ++i) {
    ros_msg.velocities.push_back(request->velocities(i));
  }

  ros_msg.duration = request->duration();

  servo_joint_msg_ = ros_msg;
  return grpc::Status::OK;
}

grpc::Status
RobotControlServiceImpl::ServoTCP(grpc::ServerContext *context,
                                  const proto_grpc_aegis::v1::Twist *request,
                                  google::protobuf::Empty *response) {

  (void)context;
  (void)response;

  const auto &lin = request->linear();
  const auto &ang = request->angular();

  auto ros_msg = geometry_msgs::msg::Twist();
  ros_msg.linear.x = lin.x();
  ros_msg.linear.y = lin.y();
  ros_msg.linear.z = lin.z();
  ros_msg.angular.x = ang.x();
  ros_msg.angular.y = ang.y();
  ros_msg.angular.z = ang.z();

  servo_tcp_msg_ = ros_msg;
  return grpc::Status::OK;
}

grpc::Status RobotControlServiceImpl::GriperSetWidth(
    grpc::ServerContext *context,
    const proto_grpc_aegis::v1::GripperSetWidthRequest *request,
    proto_grpc_aegis::v1::TriggerResponse *response) {

  (void)context;

  const double position = request->position();
  const double effort = request->effort();
  GripperSendGoal(position, effort);

  response->set_success(gripper_cmd_success_);
  response->set_msg(gripper_cmd_msg_);

  return grpc::Status::OK;
}

grpc::Status RobotControlServiceImpl::GriperClose(
    grpc::ServerContext *context, const google::protobuf::Empty *request,
    proto_grpc_aegis::v1::TriggerResponse *response) {
  (void)context;
  (void)request;
  GripperSendGoal(0.0, 0.0);
  response->set_success(gripper_cmd_success_);
  response->set_msg(gripper_cmd_msg_);
  return grpc::Status::OK;
}

grpc::Status RobotControlServiceImpl::GriperOpen(
    grpc::ServerContext *context, const google::protobuf::Empty *request,
    proto_grpc_aegis::v1::TriggerResponse *response) {

  (void)context;
  (void)request;

  // TODO define OPEN and CLOSE values as config parameters
  GripperSendGoal(0.05, 0.0);
  response->set_success(gripper_cmd_success_);
  response->set_msg(gripper_cmd_msg_);
  return grpc::Status::OK;
}

void RobotControlServiceImpl::GripperSendGoal(double position,
                                              double max_effort) {
  if (!gripper_client_->wait_for_action_server()) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Gripper Controller action server is not available. Skipping goal.");
    return;
  }

  GripperCommand::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = max_effort;

  rclcpp_action::Client<GripperCommand>::SendGoalOptions options;

  options.result_callback =
      [this](const GoalHandleGripper::WrappedResult &result) {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          gripper_cmd_success_ = true;
          gripper_cmd_msg_ = "";
          return;
        case rclcpp_action::ResultCode::ABORTED:
          gripper_cmd_msg_ = "ABORTED";
          break;
        case rclcpp_action::ResultCode::CANCELED:
          gripper_cmd_msg_ = "CANCELED";
          break;
        default:
          gripper_cmd_msg_ = "UNKNOWN";
          break;
        }
        gripper_cmd_success_ = false;
        RCLCPP_ERROR(this->node_->get_logger(), "Gripper goal result: %s",
                     gripper_cmd_msg_.c_str());
      };

  gripper_client_->async_send_goal(goal, options);
}

} // namespace aegis_grpc
