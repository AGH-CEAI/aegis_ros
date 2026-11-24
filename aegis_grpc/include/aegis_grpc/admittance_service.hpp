#ifndef AEGIS_GRPC__ADMITTANCE_SERVICE_HPP_
#define AEGIS_GRPC__ADMITTANCE_SERVICE_HPP_
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <grpcpp/grpcpp.h>
#include <google/protobuf/empty.pb.h>
#include "proto_aegis_grpc/v1/admittance_service.grpc.pb.h"

#include <Eigen/Geometry>
#include <control_msgs/msg/admittance_controller_state.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "aegis_grpc/admittance_reference.hpp"

namespace aegis_grpc {

/**
 * gRPC service for compliant TCP control through the ros2_control admittance chain
 * (admittance_jtc -> admittance_controller -> UR). See admittance_service.proto.
 */
class AdmittanceServiceImpl final : public proto_aegis_grpc::v1::AdmittanceService::Service {
 public:
  AdmittanceServiceImpl(std::shared_ptr<rclcpp::Node> node, moveit::core::RobotModelConstPtr robot_model);

  grpc::Status Enable(grpc::ServerContext* context,
                      const google::protobuf::Empty* request,
                      proto_aegis_grpc::v1::TriggerResponse* response) override;

  grpc::Status Disable(grpc::ServerContext* context,
                       const google::protobuf::Empty* request,
                       proto_aegis_grpc::v1::TriggerResponse* response) override;

  grpc::Status StepTCP(grpc::ServerContext* context,
                       const proto_aegis_grpc::v1::PoseDelta* request,
                       proto_aegis_grpc::v1::AdmittanceStepResponse* response) override;

  grpc::Status SetTCPTarget(grpc::ServerContext* context,
                            const proto_aegis_grpc::v1::Pose* request,
                            proto_aegis_grpc::v1::AdmittanceStepResponse* response) override;

  grpc::Status GetWrench(grpc::ServerContext* context,
                         const google::protobuf::Empty* request,
                         proto_aegis_grpc::v1::Wrench* response) override;

 private:
  using ControllerClaims = std::map<std::string, std::vector<std::string>>;

  // Controller manager helpers
  bool ListActiveControllers(ControllerClaims& active, std::string& err);
  bool SwitchControllers(const std::vector<std::string>& activate,
                         const std::vector<std::string>& deactivate,
                         std::string& err);
  bool ClaimsArmJoint(const std::vector<std::string>& claimed_interfaces) const;

  // Robot state helpers (call with state_mutex_ locked)
  bool GetMeasuredJoints(std::vector<double>& q, std::string& err);
  Eigen::Isometry3d ForwardKinematics(const std::vector<double>& q);
  bool SolveIK(const Eigen::Isometry3d& target,
               const std::vector<double>& seed,
               std::vector<double>& q,
               std::string& err);
  bool IsStateValid(const std::vector<double>& q, std::string& err);
  void PublishReference(const std::vector<double>& q_target, const std::vector<double>& q_previous, double duration);

  // Common implementation of StepTCP / SetTCPTarget (after the reference candidate was computed)
  void ApplyCandidate(bool clipped, proto_aegis_grpc::v1::AdmittanceStepResponse* response);

  void JointStateSubCb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void StatusSubCb(const control_msgs::msg::AdmittanceControllerState::SharedPtr msg);

  rclcpp::Logger get_logger() const;

  template <class T>
  void DeclareROSParameter(const std::string& name, const T& default_val, const std::string& description);

  std::shared_ptr<rclcpp::Node> node_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* jmg_{nullptr};
  const moveit::core::LinkModel* tip_link_{nullptr};
  std::unique_ptr<moveit::core::RobotState> robot_state_;
  std::vector<std::string> joint_names_;

  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr state_validity_client_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<control_msgs::msg::AdmittanceControllerState>::SharedPtr status_sub_;

  // Parameters
  std::vector<std::string> chain_controllers_;
  std::vector<std::string> servo_controllers_;
  std::string fallback_controller_;
  double step_duration_s_{0.1};
  double settle_time_s_{0.3};
  double max_joint_step_{0.2};
  double ik_damping_{0.01};
  int ik_max_iterations_{20};
  bool check_collisions_{true};
  std::chrono::duration<double> service_timeout_{1.0};
  std::chrono::duration<double> collision_check_timeout_{0.1};
  std::chrono::duration<double> joint_state_timeout_{0.5};

  // Reference generator state
  std::mutex state_mutex_;
  bool active_{false};
  AdmittanceReference reference_;
  std::vector<double> q_ref_;

  // Subscribed data
  std::mutex joint_state_mutex_;
  std::map<std::string, double> joint_positions_;
  rclcpp::Time joint_state_stamp_{0, 0, RCL_ROS_TIME};

  std::mutex wrench_mutex_;
  geometry_msgs::msg::Wrench wrench_;
  // The admittance controller publishes its status only while active; used to detect that the
  // chain was stopped externally (e.g. controller_stopper after a protective stop).
  rclcpp::Time status_stamp_{0, 0, RCL_ROS_TIME};
  bool IsChainRunning();
};

}  // namespace aegis_grpc
#endif  // AEGIS_GRPC__ADMITTANCE_SERVICE_HPP_
