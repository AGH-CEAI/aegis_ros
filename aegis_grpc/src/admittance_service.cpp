#include "aegis_grpc/admittance_service.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <thread>

namespace aegis_grpc {

namespace {

Eigen::Isometry3d ProtoPoseToEigen(const proto_aegis_grpc::v1::Pose& pose) {
  Eigen::Quaterniond q(pose.orientation().w(), pose.orientation().x(), pose.orientation().y(), pose.orientation().z());
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = q.normalized().toRotationMatrix();
  out.translation() = Eigen::Vector3d(pose.position().x(), pose.position().y(), pose.position().z());
  return out;
}

void EigenToProtoPose(const Eigen::Isometry3d& pose, proto_aegis_grpc::v1::Pose* out) {
  const Eigen::Quaterniond q(pose.linear());
  out->mutable_position()->set_x(pose.translation().x());
  out->mutable_position()->set_y(pose.translation().y());
  out->mutable_position()->set_z(pose.translation().z());
  out->mutable_orientation()->set_x(q.x());
  out->mutable_orientation()->set_y(q.y());
  out->mutable_orientation()->set_z(q.z());
  out->mutable_orientation()->set_w(q.w());
}

}  // namespace

AdmittanceServiceImpl::AdmittanceServiceImpl(std::shared_ptr<rclcpp::Node> node,
                                             moveit::core::RobotModelConstPtr robot_model)
    : node_(node), robot_model_(robot_model) {
  DeclareROSParameter("admittance_group", std::string("aegis_arm"),
                      "[str] Init; MoveIt planning group controlled by the admittance chain.");
  DeclareROSParameter("admittance_tcp_link", std::string("robotiq_hande_end"),
                      "[str] Init; Link whose pose is commanded (TCP).");
  DeclareROSParameter("admittance_controllers", std::vector<std::string>{"admittance_controller", "admittance_jtc"},
                      "[str[]] Init; Controllers of the admittance chain (activated together).");
  DeclareROSParameter("admittance_servo_controllers",
                      std::vector<std::string>{"forward_position_controller", "forward_velocity_controller"},
                      "[str[]] Init; Servo output controllers. Enable() is refused while one of them is active.");
  DeclareROSParameter("admittance_fallback_controller", std::string("scaled_joint_trajectory_controller"),
                      "[str] Init; Joint controller activated by Disable() (used by MoveIt).");
  DeclareROSParameter("admittance_topic_trajectory", std::string("/admittance_jtc/joint_trajectory"),
                      "[str] Init; Pub: joint trajectory topic of the chained JTC.");
  DeclareROSParameter("admittance_topic_status", std::string("/admittance_controller/status"),
                      "[str] Init; Sub: admittance controller state (wrench used by the controller).");
  DeclareROSParameter("admittance_topic_joints", std::string("/joint_states"),
                      "[str] Init; Sub: joint states used as the measured robot state.");
  DeclareROSParameter("admittance_step_duration_s", 0.1,
                      "[double] Init; Duration of each reference segment; set to the policy period.");
  DeclareROSParameter("admittance_settle_time_s", 0.3,
                      "[double] Init; Time to relax the reference to the measured pose before Disable() switches.");
  DeclareROSParameter("admittance_max_step_lin", 0.01, "[double] Init; Max TCP translation per call [m].");
  DeclareROSParameter("admittance_max_step_rot", 0.05, "[double] Init; Max TCP rotation per call [rad].");
  DeclareROSParameter("admittance_max_offset_lin", 0.02,
                      "[double] Init; Max reference-to-measured translation [m]. Bounds the force the robot applies "
                      "to ~stiffness*offset; a larger external force pushes the robot away. Keep >= 2x max_step_lin, "
                      "otherwise the tracking lag clips free-space motion.");
  DeclareROSParameter("admittance_max_accel_lin", 0.0,
                      "[double] Init; Max acceleration of the TCP reference [m/s^2]: the per-call translation may "
                      "change by at most accel * step_duration^2 between calls (0 = disabled).");
  DeclareROSParameter("admittance_max_offset_rot", 0.1,
                      "[double] Init; Max reference-to-measured rotation [rad]. Keep >= 2x max_step_rot.");
  DeclareROSParameter("admittance_workspace_min", std::vector<double>{},
                      "[double[3]] Init; Workspace box min corner [m] in the root frame (empty = disabled).");
  DeclareROSParameter("admittance_workspace_max", std::vector<double>{},
                      "[double[3]] Init; Workspace box max corner [m] in the root frame (empty = disabled).");
  DeclareROSParameter("admittance_max_joint_step", 0.2,
                      "[double] Init; Reject IK solutions moving any joint more than this per call [rad].");
  DeclareROSParameter("admittance_ik_damping", 0.01, "[double] Init; Damped least squares IK damping factor.");
  DeclareROSParameter("admittance_ik_max_iterations", 20, "[int] Init; Damped least squares IK iterations.");
  DeclareROSParameter("admittance_check_collisions", true,
                      "[bool] Init; Validate each reference with MoveIt (/check_state_validity).");
  DeclareROSParameter("admittance_collision_check_timeout_s", 0.1,
                      "[double] Init; Timeout of a single collision check [s].");

  chain_controllers_ = node_->get_parameter("admittance_controllers").as_string_array();
  servo_controllers_ = node_->get_parameter("admittance_servo_controllers").as_string_array();
  fallback_controller_ = node_->get_parameter("admittance_fallback_controller").as_string();
  step_duration_s_ = node_->get_parameter("admittance_step_duration_s").as_double();
  settle_time_s_ = node_->get_parameter("admittance_settle_time_s").as_double();
  max_joint_step_ = node_->get_parameter("admittance_max_joint_step").as_double();
  ik_damping_ = node_->get_parameter("admittance_ik_damping").as_double();
  ik_max_iterations_ = static_cast<int>(node_->get_parameter("admittance_ik_max_iterations").as_int());
  check_collisions_ = node_->get_parameter("admittance_check_collisions").as_bool();
  collision_check_timeout_ =
      std::chrono::duration<double>(node_->get_parameter("admittance_collision_check_timeout_s").as_double());

  AdmittanceReferenceLimits limits;
  limits.max_step_lin = node_->get_parameter("admittance_max_step_lin").as_double();
  limits.max_step_rot = node_->get_parameter("admittance_max_step_rot").as_double();
  limits.max_offset_lin = node_->get_parameter("admittance_max_offset_lin").as_double();
  limits.max_offset_rot = node_->get_parameter("admittance_max_offset_rot").as_double();
  const double max_accel_lin = node_->get_parameter("admittance_max_accel_lin").as_double();
  if (max_accel_lin > 0.0) {
    limits.max_step_change_lin = max_accel_lin * step_duration_s_ * step_duration_s_;
  }
  const auto ws_min = node_->get_parameter("admittance_workspace_min").as_double_array();
  const auto ws_max = node_->get_parameter("admittance_workspace_max").as_double_array();
  if (ws_min.size() == 3 && ws_max.size() == 3) {
    limits.workspace_enabled = true;
    limits.workspace_min = Eigen::Vector3d(ws_min[0], ws_min[1], ws_min[2]);
    limits.workspace_max = Eigen::Vector3d(ws_max[0], ws_max[1], ws_max[2]);
  } else if (!ws_min.empty() || !ws_max.empty()) {
    RCLCPP_WARN(get_logger(), "admittance_workspace_min/max must both have 3 elements. Workspace box disabled.");
  }
  reference_ = AdmittanceReference(limits);

  if (!robot_model_) {
    throw std::runtime_error("AdmittanceService: robot model is not available.");
  }
  const auto group_name = node_->get_parameter("admittance_group").as_string();
  jmg_ = robot_model_->getJointModelGroup(group_name);
  if (!jmg_) {
    throw std::runtime_error("AdmittanceService: unknown joint model group '" + group_name + "'.");
  }
  const auto tcp_link = node_->get_parameter("admittance_tcp_link").as_string();
  tip_link_ = robot_model_->getLinkModel(tcp_link);
  if (!tip_link_) {
    throw std::runtime_error("AdmittanceService: unknown TCP link '" + tcp_link + "'.");
  }
  joint_names_ = jmg_->getVariableNames();
  robot_state_ = std::make_unique<moveit::core::RobotState>(robot_model_);
  robot_state_->setToDefaultValues();
  robot_state_->update();

  list_controllers_client_ =
      node_->create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
  switch_controller_client_ =
      node_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
  state_validity_client_ = node_->create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");
  trajectory_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      node_->get_parameter("admittance_topic_trajectory").as_string(), 10);
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      node_->get_parameter("admittance_topic_joints").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&AdmittanceServiceImpl::JointStateSubCb, this, std::placeholders::_1));
  status_sub_ = node_->create_subscription<control_msgs::msg::AdmittanceControllerState>(
      node_->get_parameter("admittance_topic_status").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&AdmittanceServiceImpl::StatusSubCb, this, std::placeholders::_1));
}

rclcpp::Logger AdmittanceServiceImpl::get_logger() const {
  return node_->get_logger();
}

template <class T>
void AdmittanceServiceImpl::DeclareROSParameter(const std::string& name,
                                                const T& default_val,
                                                const std::string& description) {
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description = description;

  node_->declare_parameter<T>(name, default_val, param_desc);

  const auto p = node_->get_parameter(name);
  RCLCPP_INFO(get_logger(), "> %s := %s", name.c_str(), p.value_to_string().c_str());
}

void AdmittanceServiceImpl::JointStateSubCb(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  const auto n = std::min(msg->name.size(), msg->position.size());
  for (size_t i = 0; i < n; ++i) {
    joint_positions_[msg->name[i]] = msg->position[i];
  }
  joint_state_stamp_ = node_->now();
}

void AdmittanceServiceImpl::StatusSubCb(const control_msgs::msg::AdmittanceControllerState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  wrench_ = msg->wrench_base.wrench;
  status_stamp_ = node_->now();
}

bool AdmittanceServiceImpl::IsChainRunning() {
  constexpr double kStatusTimeoutS = 0.25;
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  return status_stamp_.nanoseconds() != 0 && (node_->now() - status_stamp_).seconds() < kStatusTimeoutS;
}

// ------------------------------------------------------------------------------------------------
// Controller manager helpers

bool AdmittanceServiceImpl::ClaimsArmJoint(const std::vector<std::string>& claimed_interfaces) const {
  for (const auto& interface : claimed_interfaces) {
    for (const auto& joint : joint_names_) {
      if (interface.rfind(joint + "/", 0) == 0) {
        return true;
      }
    }
  }
  return false;
}

bool AdmittanceServiceImpl::ListActiveControllers(ControllerClaims& active, std::string& err) {
  if (!list_controllers_client_->wait_for_service(service_timeout_)) {
    err = "Service /controller_manager/list_controllers not available.";
    return false;
  }
  auto future = list_controllers_client_->async_send_request(
      std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
  if (future.wait_for(service_timeout_) != std::future_status::ready) {
    err = "Service /controller_manager/list_controllers timed out.";
    return false;
  }
  active.clear();
  // Keep the response alive: future.get() returns the only owner of the response.
  const auto response = future.get();
  for (const auto& controller : response->controller) {
    if (controller.state == "active") {
      active[controller.name] = controller.claimed_interfaces;
    }
  }
  return true;
}

bool AdmittanceServiceImpl::SwitchControllers(const std::vector<std::string>& activate,
                                              const std::vector<std::string>& deactivate,
                                              std::string& err) {
  if (activate.empty() && deactivate.empty()) {
    return true;
  }
  if (!switch_controller_client_->wait_for_service(service_timeout_)) {
    err = "Service /controller_manager/switch_controller not available.";
    return false;
  }
  auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  req->activate_controllers = activate;
  req->deactivate_controllers = deactivate;
  req->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  auto future = switch_controller_client_->async_send_request(req);
  if (future.wait_for(service_timeout_ * 3) != std::future_status::ready) {
    err = "Service /controller_manager/switch_controller timed out.";
    return false;
  }
  if (!future.get()->ok) {
    err = "Controller switch rejected by the controller_manager (see its log).";
    return false;
  }
  return true;
}

// ------------------------------------------------------------------------------------------------
// Robot state helpers

bool AdmittanceServiceImpl::GetMeasuredJoints(std::vector<double>& q, std::string& err) {
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  if (joint_state_stamp_.nanoseconds() == 0 ||
      (node_->now() - joint_state_stamp_).seconds() > joint_state_timeout_.count()) {
    err = "No fresh joint states received.";
    return false;
  }
  q.resize(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto it = joint_positions_.find(joint_names_[i]);
    if (it == joint_positions_.end()) {
      err = "Joint '" + joint_names_[i] + "' missing in joint states.";
      return false;
    }
    q[i] = it->second;
  }
  return true;
}

Eigen::Isometry3d AdmittanceServiceImpl::ForwardKinematics(const std::vector<double>& q) {
  robot_state_->setJointGroupPositions(jmg_, q);
  robot_state_->update();
  return robot_state_->getGlobalLinkTransform(tip_link_);
}

bool AdmittanceServiceImpl::SolveIK(const Eigen::Isometry3d& target,
                                    const std::vector<double>& seed,
                                    std::vector<double>& q,
                                    std::string& err) {
  // Damped least squares, warm-started from the previous reference: small steps converge in
  // a few iterations and the solution stays on the same IK branch (no configuration flips).
  constexpr double kTolLin = 1e-4;  // [m]
  constexpr double kTolRot = 1e-3;  // [rad]
  const auto n = static_cast<Eigen::Index>(joint_names_.size());
  Eigen::VectorXd qv = Eigen::Map<const Eigen::VectorXd>(seed.data(), n);
  Eigen::Matrix<double, 6, 1> e;
  Eigen::MatrixXd jacobian;

  // getJacobian() expresses the Jacobian in the parent link frame of the group's first joint.
  const moveit::core::LinkModel* root_link = jmg_->getJointModels().front()->getParentLinkModel();

  for (int it = 0; it <= ik_max_iterations_; ++it) {
    robot_state_->setJointGroupPositions(jmg_, qv);
    robot_state_->enforceBounds(jmg_);
    robot_state_->update();
    robot_state_->copyJointGroupPositions(jmg_, qv);

    const Eigen::Isometry3d current = robot_state_->getGlobalLinkTransform(tip_link_);
    e.head<3>() = target.translation() - current.translation();
    e.tail<3>() = AdmittanceReference::RotationVector(target.linear() * current.linear().transpose());
    if (e.head<3>().norm() < kTolLin && e.tail<3>().norm() < kTolRot) {
      q.assign(qv.data(), qv.data() + n);
      return true;
    }
    if (it == ik_max_iterations_) {
      break;
    }

    if (!robot_state_->getJacobian(jmg_, tip_link_, Eigen::Vector3d::Zero(), jacobian)) {
      err = "Jacobian computation failed.";
      return false;
    }
    if (root_link) {
      const Eigen::Matrix3d r_root = robot_state_->getGlobalLinkTransform(root_link).linear();
      jacobian.topRows<3>() = r_root * jacobian.topRows<3>();
      jacobian.bottomRows<3>() = r_root * jacobian.bottomRows<3>();
    }
    const Eigen::Matrix<double, 6, 6> jjt =
        jacobian * jacobian.transpose() + ik_damping_ * ik_damping_ * Eigen::Matrix<double, 6, 6>::Identity();
    qv += jacobian.transpose() * jjt.ldlt().solve(e);
  }

  std::ostringstream ss;
  ss << "IK did not converge (residual " << e.head<3>().norm() * 1000.0 << " mm, " << e.tail<3>().norm()
     << " rad); target unreachable, at a joint limit or near a singularity.";
  err = ss.str();
  return false;
}

bool AdmittanceServiceImpl::IsStateValid(const std::vector<double>& q, std::string& err) {
  if (!check_collisions_) {
    return true;
  }
  if (!state_validity_client_->service_is_ready()) {
    err = "Service /check_state_validity not available (is move_group running?).";
    return false;
  }
  auto req = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
  req->group_name = jmg_->getName();
  req->robot_state.is_diff = true;
  req->robot_state.joint_state.name = joint_names_;
  req->robot_state.joint_state.position = q;
  auto future = state_validity_client_->async_send_request(req);
  if (future.wait_for(collision_check_timeout_) != std::future_status::ready) {
    err = "Collision check timed out.";
    return false;
  }
  const auto res = future.get();
  if (!res->valid) {
    err = "Reference rejected: state in collision or violating constraints";
    if (!res->contacts.empty()) {
      err += " (" + res->contacts.front().contact_body_1 + " - " + res->contacts.front().contact_body_2 + ")";
    }
    err += ".";
    return false;
  }
  return true;
}

void AdmittanceServiceImpl::PublishReference(const std::vector<double>& q_target,
                                             const std::vector<double>& q_previous,
                                             double duration) {
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);  // start immediately
  msg.joint_names = joint_names_;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = q_target;
  point.velocities.resize(q_target.size(), 0.0);
  if (duration > 0.0 && q_previous.size() == q_target.size()) {
    for (size_t i = 0; i < q_target.size(); ++i) {
      point.velocities[i] = (q_target[i] - q_previous[i]) / duration;
    }
  }
  point.time_from_start = rclcpp::Duration::from_seconds(duration);
  msg.points.push_back(point);
  trajectory_pub_->publish(msg);
}

// ------------------------------------------------------------------------------------------------
// gRPC services

grpc::Status AdmittanceServiceImpl::Enable(grpc::ServerContext* context,
                                           const google::protobuf::Empty* request,
                                           proto_aegis_grpc::v1::TriggerResponse* response) {
  (void)context;
  (void)request;
  std::lock_guard<std::mutex> lock(state_mutex_);
  response->set_success(false);
  std::string err;

  ControllerClaims active;
  if (!ListActiveControllers(active, err)) {
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::UNAVAILABLE, err);
  }
  for (const auto& servo : servo_controllers_) {
    if (active.count(servo)) {
      err = "Servo controller '" + servo + "' is active. Call ServoDisable first.";
      response->set_msg(err);
      return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, err);
    }
  }
  if (check_collisions_ && !state_validity_client_->wait_for_service(service_timeout_)) {
    err =
        "Service /check_state_validity not available (is move_group running?). "
        "Start move_group or set admittance_check_collisions:=false.";
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::UNAVAILABLE, err);
  }

  std::vector<double> q;
  if (!GetMeasuredJoints(q, err)) {
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::UNAVAILABLE, err);
  }

  // Deactivate every other active controller commanding the arm joints; activate the chain.
  std::vector<std::string> deactivate;
  for (const auto& [name, claims] : active) {
    const bool in_chain =
        std::find(chain_controllers_.begin(), chain_controllers_.end(), name) != chain_controllers_.end();
    if (!in_chain && ClaimsArmJoint(claims)) {
      deactivate.push_back(name);
    }
  }
  std::vector<std::string> activate;
  for (const auto& name : chain_controllers_) {
    if (!active.count(name)) {
      activate.push_back(name);
    }
  }
  if (!SwitchControllers(activate, deactivate, err)) {
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::INTERNAL, err);
  }

  // The chained JTC holds the current position on activation; start the reference there too.
  q_ref_ = q;
  reference_.Reset(ForwardKinematics(q));
  active_ = true;

  response->set_success(true);
  response->set_msg("Admittance chain enabled.");
  RCLCPP_INFO(get_logger(), "[AdmittanceService][Enable] Admittance chain enabled.");
  return grpc::Status::OK;
}

grpc::Status AdmittanceServiceImpl::Disable(grpc::ServerContext* context,
                                            const google::protobuf::Empty* request,
                                            proto_aegis_grpc::v1::TriggerResponse* response) {
  (void)context;
  (void)request;
  std::lock_guard<std::mutex> lock(state_mutex_);
  response->set_success(false);
  std::string err;

  // Relax: move the reference onto the measured pose, so the stiff fallback controller does not
  // take over while the spring is deflected (e.g. pressing against a surface).
  std::vector<double> q;
  if (active_ && GetMeasuredJoints(q, err)) {
    PublishReference(q, q, settle_time_s_);
    std::this_thread::sleep_for(std::chrono::duration<double>(settle_time_s_ + 0.1));
  }

  ControllerClaims active;
  if (!ListActiveControllers(active, err)) {
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::UNAVAILABLE, err);
  }
  std::vector<std::string> deactivate;
  for (const auto& name : chain_controllers_) {
    if (active.count(name)) {
      deactivate.push_back(name);
    }
  }
  std::vector<std::string> activate;
  if (!fallback_controller_.empty() && !active.count(fallback_controller_)) {
    activate.push_back(fallback_controller_);
  }
  if (!SwitchControllers(activate, deactivate, err)) {
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::INTERNAL, err);
  }

  active_ = false;
  response->set_success(true);
  response->set_msg("Admittance chain disabled.");
  RCLCPP_INFO(get_logger(), "[AdmittanceService][Disable] Admittance chain disabled.");
  return grpc::Status::OK;
}

void AdmittanceServiceImpl::ApplyCandidate(bool clipped, proto_aegis_grpc::v1::AdmittanceStepResponse* response) {
  std::string err;
  std::vector<double> q_new;
  response->set_clipped(clipped);
  response->set_success(false);

  if (!SolveIK(reference_.Candidate(), q_ref_, q_new, err)) {
    reference_.Hold();
    EigenToProtoPose(reference_.Reference(), response->mutable_reference());
    response->set_msg(err);
    return;
  }
  double max_dq = 0.0;
  for (size_t i = 0; i < q_new.size(); ++i) {
    max_dq = std::max(max_dq, std::abs(q_new[i] - q_ref_[i]));
  }
  if (max_dq > max_joint_step_) {
    reference_.Hold();
    EigenToProtoPose(reference_.Reference(), response->mutable_reference());
    response->set_msg("Rejected: joint step " + std::to_string(max_dq) + " rad exceeds admittance_max_joint_step.");
    return;
  }
  if (!IsStateValid(q_new, err)) {
    reference_.Hold();
    EigenToProtoPose(reference_.Reference(), response->mutable_reference());
    response->set_msg(err);
    return;
  }

  reference_.Commit();
  PublishReference(q_new, q_ref_, step_duration_s_);
  q_ref_ = q_new;
  EigenToProtoPose(reference_.Reference(), response->mutable_reference());
  response->set_success(true);
  response->set_msg(clipped ? "Applied with limits." : "");
}

grpc::Status AdmittanceServiceImpl::StepTCP(grpc::ServerContext* context,
                                            const proto_aegis_grpc::v1::PoseDelta* request,
                                            proto_aegis_grpc::v1::AdmittanceStepResponse* response) {
  (void)context;
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!active_) {
    const std::string err = "Admittance chain is not enabled. Call Enable first.";
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, err);
  }
  if (!IsChainRunning()) {
    active_ = false;
    const std::string err =
        "admittance_controller is not running (stopped externally, e.g. after a protective stop). "
        "Call Enable again.";
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, err);
  }
  std::string err;
  std::vector<double> q_meas;
  if (!GetMeasuredJoints(q_meas, err)) {
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::UNAVAILABLE, err);
  }
  const Eigen::Isometry3d measured = ForwardKinematics(q_meas);
  const Eigen::Vector3d dp(request->translation().x(), request->translation().y(), request->translation().z());
  const Eigen::Vector3d dr(request->rotation().x(), request->rotation().y(), request->rotation().z());
  const bool clipped = reference_.Step(dp, dr, measured);
  ApplyCandidate(clipped, response);
  return grpc::Status::OK;
}

grpc::Status AdmittanceServiceImpl::SetTCPTarget(grpc::ServerContext* context,
                                                 const proto_aegis_grpc::v1::Pose* request,
                                                 proto_aegis_grpc::v1::AdmittanceStepResponse* response) {
  (void)context;
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!active_) {
    const std::string err = "Admittance chain is not enabled. Call Enable first.";
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, err);
  }
  if (!IsChainRunning()) {
    active_ = false;
    const std::string err =
        "admittance_controller is not running (stopped externally, e.g. after a protective stop). "
        "Call Enable again.";
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, err);
  }
  if (!request->has_position() || !request->has_orientation()) {
    const std::string err = "Missing position or orientation.";
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, err);
  }
  std::string err;
  std::vector<double> q_meas;
  if (!GetMeasuredJoints(q_meas, err)) {
    response->set_msg(err);
    return grpc::Status(grpc::StatusCode::UNAVAILABLE, err);
  }
  const Eigen::Isometry3d measured = ForwardKinematics(q_meas);
  const bool clipped = reference_.StepTowards(ProtoPoseToEigen(*request), measured);
  ApplyCandidate(clipped, response);
  return grpc::Status::OK;
}

grpc::Status AdmittanceServiceImpl::GetWrench(grpc::ServerContext* context,
                                              const google::protobuf::Empty* request,
                                              proto_aegis_grpc::v1::Wrench* response) {
  (void)context;
  (void)request;
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  response->mutable_force()->set_x(wrench_.force.x);
  response->mutable_force()->set_y(wrench_.force.y);
  response->mutable_force()->set_z(wrench_.force.z);
  response->mutable_torque()->set_x(wrench_.torque.x);
  response->mutable_torque()->set_y(wrench_.torque.y);
  response->mutable_torque()->set_z(wrench_.torque.z);
  return grpc::Status::OK;
}

}  // namespace aegis_grpc
