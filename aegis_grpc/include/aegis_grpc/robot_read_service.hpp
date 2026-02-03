#ifndef AEGIS_GRPC__ROBOT_READ_SERVICE_HPP_
#define AEGIS_GRPC__ROBOT_READ_SERVICE_HPP_
#include <mutex>
#include <opencv2/opencv.hpp>

#include <grpcpp/grpcpp.h>
#include <google/protobuf/empty.pb.h>
#include "proto_aegis_grpc/v1/robot_srvs.grpc.pb.h"

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace aegis_grpc {

class RobotReadServiceImpl final
    : public proto_aegis_grpc::v1::RobotReadService::Service {
  public:
    explicit RobotReadServiceImpl(std::shared_ptr<rclcpp::Node> node);

    grpc::Status GetTCPPose(
      grpc::ServerContext* context,
      const google::protobuf::Empty* request,
      proto_aegis_grpc::v1::Pose* response
    ) override;

    grpc::Status GetWrench(
      grpc::ServerContext* context,
      const google::protobuf::Empty* request,
      proto_aegis_grpc::v1::Wrench* response
    ) override;

    grpc::Status GetJointStates(
      grpc::ServerContext* context,
      const google::protobuf::Empty* request,
      proto_aegis_grpc::v1::JointState* response
    ) override;

    grpc::Status GetCameraSceneImage(
      grpc::ServerContext* context,
      const google::protobuf::Empty* request,
      proto_aegis_grpc::v1::Image* response
    ) override;

    grpc::Status GetCameraRightImage(
      grpc::ServerContext* context,
      const google::protobuf::Empty* request,
      proto_aegis_grpc::v1::Image* response
    ) override;

    grpc::Status GetCameraLeftImage(
      grpc::ServerContext* context,
      const google::protobuf::Empty* request,
      proto_aegis_grpc::v1::Image* response
    ) override;

    grpc::Status GetRobotState(
        grpc::ServerContext* context,
        const google::protobuf::Empty* request,
        proto_aegis_grpc::v1::RobotState* response
    ) override;

    grpc::Status GetRobotVision(
        grpc::ServerContext* context,
        const google::protobuf::Empty* request,
        proto_aegis_grpc::v1::RobotVision* response
    ) override;

    grpc::Status GetAll(
        grpc::ServerContext* context,
        const google::protobuf::Empty* request,
        proto_aegis_grpc::v1::RobotObservation* response
    ) override;

    // TODO(issue#84) implement getters for images from cameras (RGB & RGBD)
    // TODO(issue#85) develop synchronization guard to monitor the freshness of the data

  private:
    template <class T>
    void DeclareROSParameter(const std::string& name, const T& default_val, const std::string& description);

    void PoseSubCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void WrenchSubCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    void JointStateSubCb(const sensor_msgs::msg::JointState::SharedPtr msg);
    void ImageSceneSubCb(const sensor_msgs::msg::Image::SharedPtr msg);
    void ImageRightSubCb(const sensor_msgs::msg::Image::SharedPtr msg);
    void ImageLeftSubCb(const sensor_msgs::msg::Image::SharedPtr msg);

  static void FillProtoPose(
    const geometry_msgs::msg::Pose& ros,
    proto_aegis_grpc::v1::Pose* out);

  static void FillProtoWrench(
    const geometry_msgs::msg::Wrench& ros,
    proto_aegis_grpc::v1::Wrench* out);

  static void FillProtoJointState(
      const sensor_msgs::msg::JointState& ros,
      proto_aegis_grpc::v1::JointState* out);

  void FillProtoImage(
    const sensor_msgs::msg::Image& ros,
    proto_aegis_grpc::v1::Image* out,
    cv::Mat& resize_buffer); 

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_scene_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_right_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_left_sub_;

    geometry_msgs::msg::Pose pose_data_;
    geometry_msgs::msg::Wrench wrench_data_;
    sensor_msgs::msg::JointState joint_state_data_;
    sensor_msgs::msg::Image image_scene_data_;
    sensor_msgs::msg::Image image_right_data_;
    sensor_msgs::msg::Image image_left_data_;

    std::mutex pose_mutex_;
    std::mutex wrench_mutex_;
    std::mutex joint_state_mutex_;
    std::mutex image_scene_mutex_;
    std::mutex image_right_mutex_;
    std::mutex image_left_mutex_;

    uint32_t target_img_width_;
    uint32_t target_img_height_;

    cv::Mat scene_resized_;
    cv::Mat right_resized_;
    cv::Mat left_resized_;

    bool enable_image_resize_;
};

} // namespace aegis_grpc
#endif  // AEGIS_GRPC__ROBOT_READ_SERVICE_HPP_
