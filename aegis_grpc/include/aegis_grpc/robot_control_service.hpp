#ifndef AEGIS_GRPC__ROBOT_CONTROL_SERVICE_HPP_
#define AEGIS_GRPC__ROBOT_CONTROL_SERVICE_HPP_
#include <atomic>
#include <chrono>

#include <grpcpp/grpcpp.h>
#include <google/protobuf/empty.pb.h>
#include "proto_aegis_grpc/v1/robot_srvs.grpc.pb.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>


namespace aegis_grpc {

class RobotControlServiceImpl final
    : public proto_aegis_grpc::v1::RobotControlService::Service {
  public:
    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

    enum class ServoMode {
      None,
      JointJog,
      TCPTwist
    };

    explicit RobotControlServiceImpl(std::shared_ptr<rclcpp::Node> node);

    // TODO(issue#87) Create additional methods to enable and disable servo control.
    grpc::Status ServoJoint(
        grpc::ServerContext* context,
        const proto_aegis_grpc::v1::JointJog *request,
        google::protobuf::Empty *response) override;

    grpc::Status ServoTCP(
        grpc::ServerContext* context,
        const proto_aegis_grpc::v1::Twist *request,
        google::protobuf::Empty *response) override;

    // TODO(issue#83) add MoveIt2 actions to plan&execute trajectories to targets in Joints and Poses.
    // grpc::Status GotoPose(
    //     grpc::ServerContext* context,
    //     const proto_aegis_grpc::v1::Pose *request,
    //     proto_aegis_grpc::v1::TriggerResponse *response) override;

    // grpc::Status GotoJoints(
    //     grpc::ServerContext* context,
    //     const proto_aegis_grpc::v1::JointState *request,
    //     proto_aegis_grpc::v1::TriggerResponse *response) override;

    grpc::Status GripperSetPosition(
      grpc::ServerContext* context,
      const proto_aegis_grpc::v1::GripperSetPositionRequest *request,
      proto_aegis_grpc::v1::TriggerResponse *response) override;

    grpc::Status GripperClose(
        grpc::ServerContext* context,
        const google::protobuf::Empty *request,
        proto_aegis_grpc::v1::TriggerResponse *response) override;

    grpc::Status GripperOpen(
        grpc::ServerContext* context,
        const google::protobuf::Empty *request,
        proto_aegis_grpc::v1::TriggerResponse *response) override;

  private:
    rclcpp::Logger get_logger() const;

    template <class T>
    void DeclareROSParameter(const std::string& name, const T& default_val, const std::string& description);

    void ServoPublishLoop();
    void GripperSendGoal(double position, double max_effort);

    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr servo_joint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_tcp_pub_;
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
    rclcpp::TimerBase::SharedPtr servo_pub_timer_;

    std::mutex servo_mutex_;
    ServoMode servo_mode_;
    std::string servo_tcp_link_;
    control_msgs::msg::JointJog servo_joint_msg_;
    geometry_msgs::msg::Twist servo_tcp_msg_;
    int servo_frequency_ratio_;
    int servo_msgs_left_;

    std::mutex gripper_mutex_;
    std::atomic<bool> gripper_in_use_;
    std::chrono::duration<double> action_timeout_;
    std::string gripper_cmd_msg_;
    bool gripper_cmd_success_;
    std::atomic<bool> gripper_cmd_done_;
};

} // namespace aegis_grpc
#endif  // AEGIS_GRPC__ROBOT_CONTROL_SERVICE_HPP_
