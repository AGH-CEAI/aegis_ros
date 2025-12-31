#include "aegis_grpc/robot_read_service.hpp"

RobotReadServiceImpl::RobotReadServiceImpl(std::shared_ptr<rclcpp::Node> node) : node_(node), pose_data_(), wrench_data_(), joint_state_data_()   {
        pose_sub_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/TODO", 10, std::bind(&RobotReadServiceImpl::pose_sub_cb_, this, std::placeholders::_1));
        wrench_sub_ = node_->create_subscription<geometry_msgs::msg::Wrench>(
            "/TODO", 10, std::bind(&RobotReadServiceImpl::wrench_sub_cb_, this, std::placeholders::_1));
        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/TODO", 10, std::bind(&RobotReadServiceImpl::joint_state_sub_cb_, this, std::placeholders::_1));
}

// class RobotReadServiceImpl final : public proto_grpc_aegis::v1::RobotReadService::Service {
// public:
//     explicit RobotReadServiceImpl(std::shared_ptr<rclcpp::Node> node)
//         : node_(node), get_float32_data(0.0) {
//         publisher_ = node_->create_publisher<std_msgs::msg::Float32>("/float32_topic", 10);
//         publish_thread_ = std::thread(&Float32ServiceImpl::publishLoop, this);
//         // ROS 2 Subscriber
//         subscriber_ = node_->create_subscription<std_msgs::msg::Float32>(
//             "/data", 10, std::bind(&Float32ServiceImpl::float32Callback, this, std::placeholders::_1));
//     }

//     /******************************************************************************
//      * @fn      GetFloat32
//      * @brief   Handles incoming gRPC requests to get the Float32 data from ROS 2 topic
//      * @param   context : The gRPC server context
//      * @param   request : The request object
//      * @param   response: The response object containing the float32 data
//      * @return  grpc::Status : The status of the gRPC request
//      ******************************************************************************/
//     grpc::Status GetFloat32(
//       grpc::ServerContext* context,
//       const robot_service::Float32Request* request,
//       robot_service::Float32Response* response
//     ) override {
//         // Set the response with the current float32 data
//         response->mutable_float_data()->set_data(get_float32_data);

//         return grpc::Status::OK;
//     }

//     /******************************************************************************
//      * @fn      SetFloat32
//      * @brief   Handles incoming gRPC requests to set the Float32 data to a ROS 2 topic
//      * @param   context : The gRPC server context
//      * @param   request : The request object containing float32 data
//      * @param   response: The response object confirming the set data
//      * @return  grpc::Status : The status of the gRPC request
//      ******************************************************************************/
//     grpc::Status SetFloat32(
//       grpc::ServerContext* context,
//       const robot_service::Float32Request* request,
//       robot_service::Float32Response* response) override {

//         // Extract the float32 data from request
//         float new_data = request->float_data().data();

//         // Set the response with the updated new_data
//         response->mutable_float_data()->set_data(new_data);

//         // Update float 32 data
//         set_float32_data = new_data;

//         return grpc::Status::OK;
//     }

// private:
//     /******************************************************************************
//      * @fn      publishLoop
//      * @brief   Publishes the internal pose data to a ROS 2 topic at a fixed rate (1Hz)
//      * @detail  This method runs in a separate thread and periodically publishes
//      *          the current pose as a formatted string to the "pose_topic".
//      ******************************************************************************/
//     void publishLoop() {
//       rclcpp::Rate rate(1); // Publish at 1Hz
//       while (rclcpp::ok()) {
//           // Create and publish a ROS 2 message
//           std_msgs::msg::Float32 ros_message;
//           ros_message.data = set_float32_data;
//           publisher_->publish(ros_message);

//           // Log the published pose
//           RCLCPP_INFO(node_->get_logger(), "Published data: %f", ros_message.data);
//           rate.sleep(); // Wait 1 second
//       }
//     }

//     /******************************************************************************
//      * @fn      float32Callback
//      * @brief   ROS 2 subscription callback to update internal float32 state
//      * @param   msg : The incoming ROS 2 message with float32 data
//      ******************************************************************************/
//     void float32Callback(const std_msgs::msg::Float32::SharedPtr msg) {
//         // Update internal state from ROS 2 message
//         get_float32_data = msg->data;
//         RCLCPP_INFO(node_->get_logger(), "Received float32 data: %f", get_float32_data);
//     }

//     std::shared_ptr<rclcpp::Node> node_;
//     rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
//     rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

//     // TODO add mutexes/atomic access to these variables
//     geometry_msgs::msg::Pose pose_data_;
//     geometry_msgs::msg::Wrench wrench_data_;
//     sensor_msgs::msg::JointState joint_state_data_;
// };
