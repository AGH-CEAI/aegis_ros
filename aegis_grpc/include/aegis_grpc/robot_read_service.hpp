#include <grpcpp/grpcpp.h>
#include <google/protobuf/empty.pb.h>
#include "proto_grpc_aegeis/v1/robot_srvs.grpc.pb.h"

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/Pose.hpp"
#include "geometry_msgs/msg/Wrench.hpp"
#include "sensor_msgs/msg/JointState.hpp"

class RobotReadServiceImpl final : public proto_grpc_aegis::v1::RobotReadService::Service {
public:
    explicit RobotReadServiceImpl(std::shared_ptr<rclcpp::Node> node);

    grpc::Status GetTCPPose(
      grpc::ServerContext* context,
      const google::protobuf::Empty* request,
      proto_grpc_aegis::v1::Pose* response
    ) override;

    grpc::Status GetWrench(
      grpc::ServerContext* context,
      const google::protobuf::Empty* request,
      proto_grpc_aegis::v1::Wrench* response) override {
    }

    grpc::Status GetJointState(
      grpc::ServerContext* context,
      const google::protobuf::Empty* request,
      proto_grpc_aegis::v1::JointState* response) override {
    }

private:
    void pose_sub_cb_(const geometry_msgs::msg::Pose::SharedPtr msg);
    void wrench_sub_cb_(const geometry_msgs::msg::Wrench::SharedPtr msg);
    void joint_state_sub_cb_(const sensor_msgs::msg::JointState::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    // TODO add mutexes/atomic access to these variables
    geometry_msgs::msg::Pose pose_data_;
    geometry_msgs::msg::Wrench wrench_data_;
    sensor_msgs::msg::JointState joint_state_data_;
};
