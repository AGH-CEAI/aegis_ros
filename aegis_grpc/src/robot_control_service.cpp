#include <thread>
#include "aegis_grpc/robot_control_service.hpp"

using namespace std::chrono_literals;

namespace aegis_grpc {

RobotControlServiceImpl::RobotControlServiceImpl(
    std::shared_ptr<rclcpp::Node> node)
    : node_(node), servo_mode_(ServoMode::None), servo_frequency_ratio_(0), servo_msgs_left_(0), gripper_in_use_(false),
      action_timeout_(0.0), gripper_cmd_success_(false), gripper_cmd_done_(false) {

  // Initialization parameters
  DeclareROSParameter("servo_link", std::string("base_link"),
                      "[str] Init; Name of the base link for the TCP servoing.");
  DeclareROSParameter("topic_servo_joint", std::string("/servo_node/delta_joint_cmds"),
                      "[str] Init; Pub: output topic for joints servo commands.");
  DeclareROSParameter("topic_servo_tcp", std::string("/servo_node/delta_twist_cmds"),
                      "[str] Init; Pub: output topic for TCP servo commands.");
  DeclareROSParameter("action_gripper", std::string("/gripper_action_controller/gripper_cmd"),
                      "[str] Init; Action: GripperCommand action.");
  DeclareROSParameter("action_timeout_s", 3.0, "[double] Init; Waiting timeout for action in seconds.");
  DeclareROSParameter("servo_in_rate_hz", 10.0, "[double] Init; Servo commands frequency in Hz.");
  DeclareROSParameter("servo_out_rate_hz", 250.0, "[double] Init; Servo publish loop frequency in Hz.");
  DeclareROSParameter("move_group", std::string("aegis_arm"), "[str] Init; Name of the planning group to control.");

  // Runtime parameters
  DeclareROSParameter("r_gripper_close_m", 0.0, "[double] Runtime; Gripper close position in meters. ");
  DeclareROSParameter("r_gripper_open_m", 0.025, "[double] Runtime; Gripper open position in meters.");

  servo_tcp_link_ = node_->get_parameter("servo_link").as_string();

  auto servo_in_hz = node_->get_parameter("servo_in_rate_hz").as_double();
  auto servo_out_hz = node_->get_parameter("servo_out_rate_hz").as_double();
  servo_frequency_ratio_ = std::round(servo_out_hz / servo_in_hz);
  RCLCPP_INFO(get_logger(), "> Frequency ratio for servo re-publishig is %i", servo_frequency_ratio_);

  switch_controller_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
  start_servo_client_ = node_->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
  stop_servo_client_ = node_->create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");
  servo_joint_pub_ = node_->create_publisher<control_msgs::msg::JointJog>(
      node_->get_parameter("topic_servo_joint").as_string(), 10);
  servo_tcp_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      node_->get_parameter("topic_servo_tcp").as_string(), 10);
  gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      node_, node_->get_parameter("action_gripper").as_string());

  double hz = node_->get_parameter("servo_out_rate_hz").as_double();
  auto servo_pub_period = std::chrono::duration<double>(1.0 / hz);

  servo_pub_timer_ = node_->create_wall_timer(
      servo_pub_period, std::bind(&RobotControlServiceImpl::ServoPublishLoop, this));

  auto action_timeout = node_->get_parameter("action_timeout_s").as_double();
  action_timeout_ = std::chrono::duration<double>(action_timeout);

  auto move_group_name = node_->get_parameter("move_group").as_string();
  move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, move_group_name);
}

rclcpp::Logger RobotControlServiceImpl::get_logger() const {
  return node_->get_logger();
}

template <class T>
void RobotControlServiceImpl::DeclareROSParameter(
    const std::string &name,
    const T& default_val,
    const std::string &description) {
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description = description;

  node_->declare_parameter<T>(name, default_val, param_desc);

  const auto p = node_->get_parameter(name);
  RCLCPP_INFO(get_logger(), "> %s := %s",
              name.c_str(),
              p.value_to_string().c_str());
}

bool RobotControlServiceImpl::SwitchControllers(
  const std::vector<std::string>& activate,
  const std::vector<std::string>& deactivate) {
  auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();

  req->activate_controllers = activate;
  req->deactivate_controllers = deactivate;
  req->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

  if (!switch_controller_client_->wait_for_service(action_timeout_)) {
    RCLCPP_WARN(get_logger(), "Switch controller service not available");
    return false;
  }

  auto future = switch_controller_client_->async_send_request(req);
  auto status = future.wait_for(action_timeout_);

  if (status != std::future_status::ready || !future.valid()) {
    RCLCPP_WARN(get_logger(), "Switch controller call failed or timed out.");
    return false;
  }

  return future.get()->ok;
}

bool RobotControlServiceImpl::CallServoStartService() {
  if (!start_servo_client_->wait_for_service(action_timeout_)) {
    RCLCPP_WARN(get_logger(), "Service `/servo_node/start_servo` not available.");
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result_future = start_servo_client_->async_send_request(request);

  if (result_future.wait_for(action_timeout_) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "Service `/servo_node/start_servo` call timed out.");
    return false;
  }
  auto result = result_future.get();
  return result->success;
}

bool RobotControlServiceImpl::CallServoStopService() {
  if (!stop_servo_client_->wait_for_service(action_timeout_)) {
    RCLCPP_WARN(get_logger(), "Service `/servo_node/stop_servo` not available.");
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result_future = stop_servo_client_->async_send_request(request);

  if (result_future.wait_for(action_timeout_) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "Service `/servo_node/stop_servo` call timed out.");
    return false;
  }
  auto result = result_future.get();
  return result->success;
}

grpc::Status RobotControlServiceImpl::ServoEnable(
  grpc::ServerContext*,
  const google::protobuf::Empty*,
  proto_aegis_grpc::v1::TriggerResponse* response)
{
  response->set_success(false);

  if (!SwitchControllers(
        {"forward_position_controller"},
        {"scaled_joint_trajectory_controller"})) {
    std::string err = "Controller switch failed.";
    RCLCPP_WARN(get_logger(), "[RobotControlService][ServoEnable] %s", err.c_str());
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::INTERNAL, err);
  }

  if (!CallServoStartService()) {
    std::string err = "Failed to start servo service.";
    RCLCPP_WARN(get_logger(), "[RobotControlService][ServoEnable] %s", err.c_str());
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::INTERNAL, err);
  }

  {
    std::lock_guard<std::mutex> lock(servo_mutex_);
    servo_mode_ = ServoMode::JointJog;
  }

  response->set_success(true);
  response->set_msg("");
  RCLCPP_INFO(get_logger(), "[RobotControlService][ServoEnable] Servo enabled");
  return grpc::Status::OK;
}

grpc::Status RobotControlServiceImpl::ServoDisable(
  grpc::ServerContext*,
  const google::protobuf::Empty*,
  proto_aegis_grpc::v1::TriggerResponse* response)
{
  response->set_success(false);

  if (!CallServoStopService()) {
    std::string err = "Failed to stop servo service.";
    RCLCPP_WARN(get_logger(), "[RobotControlService][ServoDisable] %s", err.c_str());
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::INTERNAL, err);
  }

  if (!SwitchControllers(
        {"scaled_joint_trajectory_controller"},
        {"forward_position_controller"})) {

    std::string err = "Controller switch failed.";
    RCLCPP_WARN(get_logger(), "[RobotControlService][ServoDisable] %s", err.c_str());
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::INTERNAL, err);
  }

  {
    std::lock_guard<std::mutex> lock(servo_mutex_);
    servo_mode_ = ServoMode::None;
    servo_msgs_left_ = 0;
  }

  response->set_success(true);
  response->set_msg("");
  RCLCPP_INFO(get_logger(), "[RobotControlService][ServoDisable] Servo disabled");
  return grpc::Status::OK;
}

void RobotControlServiceImpl::ServoPublishLoop() {
  static ServoMode mode(ServoMode::TCPTwist);
  static control_msgs::msg::JointJog jog_msg;
  static geometry_msgs::msg::TwistStamped twist_msg;

  {
    std::lock_guard<std::mutex> lock(servo_mutex_);
    mode = servo_mode_;
    if(servo_msgs_left_ == 0) {
      switch(mode) {
        case ServoMode::JointJog:
          jog_msg = servo_joint_msg_zeros_;
          break;
        case ServoMode::TCPTwist:
          twist_msg.twist = servo_tcp_msg_zeros_;
          break;
        default:
          break;
      }
    } else {
      switch(mode) {
        case ServoMode::JointJog:
          jog_msg = servo_joint_msg_;
          break;
        case ServoMode::TCPTwist:
          twist_msg.twist = servo_tcp_msg_;
          break;
        default:
          break;
      }
      servo_msgs_left_--;
    }
  }

  switch(mode) {
    case ServoMode::JointJog:
      jog_msg.header.stamp = node_->now();
      servo_joint_pub_->publish(jog_msg);
      return;

    case ServoMode::TCPTwist:
      twist_msg.header.stamp = node_->now();
      twist_msg.header.frame_id = servo_tcp_link_;
      servo_tcp_pub_->publish(twist_msg);
      return;

    default:
      return;
  }
}

grpc::Status RobotControlServiceImpl::ServoJoint(
    grpc::ServerContext *context, const proto_aegis_grpc::v1::JointJog *request,
    google::protobuf::Empty *response) {

  (void)context;
  (void)response;

  auto N = request->joint_names_size();
  if (N != request->displacements_size()) {
    std::string err =
        "The number of joints mismatches the number of displacmenets values!";
    RCLCPP_WARN(get_logger(), "[RobotControlService][ServoJoint] %s",
                err.c_str());
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, err);
  }
  if (N != request->velocities_size()) {
    std::string err =
        "The number of joints mismatches the number of velocities values!";
    RCLCPP_WARN(get_logger(), "[RobotControlService][ServoJoint] %s",
                err.c_str());
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, err);
  }

  auto ros_msg = control_msgs::msg::JointJog();

  ros_msg.joint_names.reserve(request->joint_names_size());
  ros_msg.displacements.reserve(request->displacements_size());
  ros_msg.velocities.reserve(request->velocities_size());
  for (int i = 0; i < N; i++) {
    ros_msg.joint_names.push_back(request->joint_names(i));
    ros_msg.displacements.push_back(request->displacements(i));
    ros_msg.velocities.push_back(request->velocities(i));
  }

  ros_msg.duration = request->duration();

  {
    std::lock_guard<std::mutex> lock(servo_mutex_);
    servo_mode_ = ServoMode::JointJog;
    servo_joint_msg_ = ros_msg;
    servo_msgs_left_ = servo_frequency_ratio_;
  }
  return grpc::Status::OK;
}

grpc::Status
RobotControlServiceImpl::ServoTCP(grpc::ServerContext *context,
                                  const proto_aegis_grpc::v1::Twist *request,
                                  google::protobuf::Empty *response) {

  (void)context;
  (void)response;
  static geometry_msgs::msg::Twist ros_msg;

  const auto &lin = request->linear();
  const auto &ang = request->angular();

  ros_msg.linear.x = lin.x();
  ros_msg.linear.y = lin.y();
  ros_msg.linear.z = lin.z();
  ros_msg.angular.x = ang.x();
  ros_msg.angular.y = ang.y();
  ros_msg.angular.z = ang.z();

  {
    std::lock_guard<std::mutex> lock(servo_mutex_);
    servo_mode_ = ServoMode::TCPTwist;
    servo_tcp_msg_ = ros_msg;
    servo_msgs_left_ = servo_frequency_ratio_;
  }
  return grpc::Status::OK;
}

grpc::Status RobotControlServiceImpl::GripperSetPosition(
    grpc::ServerContext *context,
    const proto_aegis_grpc::v1::GripperSetPositionRequest *request,
    proto_aegis_grpc::v1::TriggerResponse *response) {

  (void)context;

  {
    std::lock_guard<std::mutex> lock(gripper_mutex_);
    if(gripper_in_use_.load(std::memory_order_relaxed)){
      const std::string msg = "Gripper is already in use.";
      response->set_success(false);
      response->set_msg(msg);
      return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED, msg);
    }
    gripper_in_use_.store(true, std::memory_order_relaxed);
  }


  const double position = request->position();
  const double effort = request->effort();
  GripperSendGoal(position, effort);

  response->set_success(gripper_cmd_success_);
  response->set_msg(gripper_cmd_msg_);

  gripper_in_use_.store(false, std::memory_order_relaxed);
  return grpc::Status::OK;
}

grpc::Status RobotControlServiceImpl::GripperClose(
    grpc::ServerContext *context, const google::protobuf::Empty *request,
    proto_aegis_grpc::v1::TriggerResponse *response) {
  (void)context;
  (void)request;

  {
    std::lock_guard<std::mutex> lock(gripper_mutex_);
    if(gripper_in_use_.load(std::memory_order_relaxed)){
      const std::string msg = "Gripper is already in use.";
      response->set_success(false);
      response->set_msg(msg);
      return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED, msg);
    }
    gripper_in_use_.store(true, std::memory_order_relaxed);
  }

  double pos = node_->get_parameter("r_gripper_close_m").as_double();
  GripperSendGoal(pos, 0.0);

  response->set_success(gripper_cmd_success_);
  response->set_msg(gripper_cmd_msg_);
  gripper_in_use_.store(false, std::memory_order_relaxed);
  return grpc::Status::OK;
}

grpc::Status RobotControlServiceImpl::GripperOpen(
    grpc::ServerContext *context, const google::protobuf::Empty *request,
    proto_aegis_grpc::v1::TriggerResponse *response) {

  (void)context;
  (void)request;

  {
    std::lock_guard<std::mutex> lock(gripper_mutex_);
    if(gripper_in_use_.load(std::memory_order_relaxed)){
      const std::string msg = "Gripper is already in use.";
      response->set_success(false);
      response->set_msg(msg);
      return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED, msg);
    }
    gripper_in_use_.store(true, std::memory_order_relaxed);
  }

  double pos = node_->get_parameter("r_gripper_open_m").as_double();
  GripperSendGoal(pos, 0.0);

  response->set_success(gripper_cmd_success_);
  response->set_msg(gripper_cmd_msg_);
  gripper_in_use_.store(false, std::memory_order_relaxed);
  return grpc::Status::OK;
}

grpc::Status RobotControlServiceImpl::GotoPose(
  grpc::ServerContext* context,
  const proto_aegis_grpc::v1::Pose *request,
  proto_aegis_grpc::v1::TriggerResponse *response) {
  (void)context;

  response->set_success(false);

  if (!request->has_position() || !request->has_orientation()) {
    std::string err = "Missing position or orientation.";
    RCLCPP_ERROR(get_logger(), "[RobotControlService][GotoPose] %s", err.c_str());
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, err);
  }

  // TODO remove after debug
  [&]{
      char request_str[512];
      snprintf(request_str, sizeof(request_str),
          "Pose(position={x:%.3f, y:%.3f, z:%.3f}, orientation={x:%.4f, y:%.4f, z:%.4f, w:%.4f})",
          request->position().x(), request->position().y(), request->position().z(),
          request->orientation().x(), request->orientation().y(),
          request->orientation().z(), request->orientation().w());
      RCLCPP_INFO(get_logger(), "[RobotControlService][GotoPose] %s", request_str);
  }();
  // -------

  auto const target_pose = [&request]{
    geometry_msgs::msg::Pose goal;
    goal.position.x = request->position().x();
    goal.position.y = request->position().y();
    goal.position.z = request->position().z();
    goal.orientation.x = request->orientation().x();
    goal.orientation.y = request->orientation().y();
    goal.orientation.z = request->orientation().z();
    goal.orientation.w = request->orientation().w();
    return goal;
  }();
  move_group_->setPoseTarget(target_pose);

  auto const [success, plan] = [&]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_->plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (!success) {
    std::string err = "Planning failed.";
    RCLCPP_ERROR(get_logger(), "[RobotControlService][GotoPose] %s", err.c_str());
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::INTERNAL, err);
  }

  auto const exec = static_cast<bool>(move_group_->execute(plan));

  if(!exec) {
      std::string err = "Execution failed.";
      RCLCPP_ERROR(get_logger(), "[RobotControlService][GoToPose] %s", err.c_str());
      response->set_msg(err);
      return grpc::Status(grpc::StatusCode::INTERNAL, err);
  }

  response->set_success(true);
  response->set_msg("");
  return grpc::Status::OK;
}

grpc::Status RobotControlServiceImpl::GotoJoints(
  grpc::ServerContext* context,
  const proto_aegis_grpc::v1::JointState *request,
  proto_aegis_grpc::v1::TriggerResponse *response) {
  (void)context;

  response->set_success(false);

  if (request->name_size() == 0 || request->name_size() != request->position_size()) {
    std::string err = "Joint names and positions mismatch or empty.";
    RCLCPP_WARN(get_logger(), "[RobotControlService][GotoJoints] %s", err.c_str());
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, err);
  }

  auto const target = [&request]{
    std::map<std::string, double> joints_goal;
    for (int i = 0; i < request->name_size(); ++i) {
      joints_goal[request->name(i)] = request->position(i);
    }
    return joints_goal;
  }();
  move_group_->setJointValueTarget(target);

  auto const [success, plan] = [&]{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto const ok = static_cast<bool>(move_group_->plan(plan));
    return std::make_pair(ok, plan);
  }();

  if (!success) {
    std::string err = "Planning failed.";
    RCLCPP_WARN(get_logger(), "[RobotControlService][GotoJoints] %s", err.c_str());
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::INTERNAL, err);
  }

  auto const exec = static_cast<bool>(move_group_->execute(plan));

  if(!exec) {
      std::string err = "Execution failed.";
      RCLCPP_WARN(get_logger(), "[RobotControlService][GotoJoints] %s", err.c_str());
      response->set_msg(err);
      return grpc::Status(grpc::StatusCode::INTERNAL, err);
  }

  response->set_success(true);
  response->set_msg("");
  return grpc::Status::OK;
}

void RobotControlServiceImpl::GripperSendGoal(double position,
                                              double max_effort) {
  if (!gripper_client_->wait_for_action_server(action_timeout_)) {
    gripper_cmd_success_ = false;
    gripper_cmd_msg_ = "Gripper Controller action server is not available. Skipping goal.";
    RCLCPP_ERROR(get_logger(), "%s", gripper_cmd_msg_.c_str());
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
          gripper_cmd_done_.store(true, std::memory_order_relaxed);
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
        gripper_cmd_done_.store(true, std::memory_order_relaxed);
        RCLCPP_ERROR(get_logger(), "Gripper goal result: %s",
                     gripper_cmd_msg_.c_str());
      };

  gripper_cmd_done_.store(false, std::memory_order_relaxed);
  gripper_client_->async_send_goal(goal, options);

  while(!gripper_cmd_done_.load(std::memory_order_relaxed)){
    std::this_thread::sleep_for(1ms);
  }
}

} // namespace aegis_grpc
