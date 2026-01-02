#ifndef AEGIS_GRPC__ROBOT_READ_SERVICE_HPP_
#define AEGIS_GRPC__ROBOT_READ_SERVICE_HPP_
#include <grpcpp/grpcpp.h>
#include <google/protobuf/empty.pb.h>
#include "proto_aegis_grpc/v1/robot_srvs.grpc.pb.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace aegis_grpc {

class RobotReadServiceImpl final
    : public proto_aegis_grpc::v1::RobotReadService::Service {
  public:
    explicit RobotReadServiceImpl(std::shared_ptr<rclcpp::Node> node);

    grpc::Status GetTCPPose(grpc::ServerContext *context,
                            const google::protobuf::Empty *request,
                            proto_aegis_grpc::v1::Pose *response) override;

    grpc::Status GetWrench(grpc::ServerContext *context,
                          const google::protobuf::Empty *request,
                          proto_aegis_grpc::v1::Wrench *response) override;

    grpc::Status
    GetJointState(grpc::ServerContext *context,
                  const google::protobuf::Empty *request,
                  proto_aegis_grpc::v1::JointState *response) override;

    grpc::Status GetAll(grpc::ServerContext *context,
                        const google::protobuf::Empty *request,
                        proto_aegis_grpc::v1::RobotState *response) override;

  private:
    void PoseSubCb(const geometry_msgs::msg::Pose::SharedPtr msg);
    void WrenchSubCb(const geometry_msgs::msg::Wrench::SharedPtr msg);
    void JointStateSubCb(const sensor_msgs::msg::JointState::SharedPtr msg);

  static void FillProtoPose(
      const geometry_msgs::msg::Pose& ros,
      proto_aegis_grpc::v1::Pose* out);

  static void FillProtoWrench(
      const geometry_msgs::msg::Wrench& ros,
      proto_aegis_grpc::v1::Wrench* out);

  static void FillProtoJointState(
      const sensor_msgs::msg::JointState& ros,
      proto_aegis_grpc::v1::JointState* out);

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_sub_;

    // TODO add mutexes/atomic access to these variables
    geometry_msgs::msg::Pose pose_data_;
    geometry_msgs::msg::Wrench wrench_data_;
    sensor_msgs::msg::JointState joint_state_data_;
};

} // namespace aegis_grpc
#endif  // AEGIS_GRPC__ROBOT_READ_SERVICE_HPP_
