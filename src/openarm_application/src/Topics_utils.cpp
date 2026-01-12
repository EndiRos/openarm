#include "Topics_utils.hpp"

ReadOnce::ReadOnce(rclcpp::Node::SharedPtr node) : node_ (node){}

geometry_msgs::msg::Pose ReadOnce::ReadPose(std::string topic_name, double timeout_sec)
{
    // Llama al template con el tipo Pose
    return ReadMsgOnce<geometry_msgs::msg::Pose>(topic_name, timeout_sec);
}

tf2_msgs::msg::TFMessage ReadOnce::ReadTf(std::string topic_name, double timeout_sec)
{
    return ReadMsgOnce<tf2_msgs::msg::TFMessage>(topic_name, timeout_sec);
}

