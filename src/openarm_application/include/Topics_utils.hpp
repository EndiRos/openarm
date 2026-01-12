

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <future>
#include <chrono>

class ReadOnce
{
    public:
        ReadOnce(rclcpp::Node::SharedPtr node);
        geometry_msgs::msg::Pose ReadPose(std::string topic_name, double timeout_sec);
        tf2_msgs::msg::TFMessage ReadTf(std::string topic_name, double timeout_sec);
    private:
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr   tf_subscription_;
        rclcpp::Node::SharedPtr                                     node_;

        template <typename MsgT>
        MsgT ReadMsgOnce(std::string topic_name, double timeout_sec = 5.0)
        {
            auto promise = std::make_shared<std::promise<MsgT>>();
            auto future = promise->get_future();

            RCLCPP_INFO(node_->get_logger(),"Waiting mensagge from %s topic..." , topic_name.c_str());

            typename rclcpp::Subscription<MsgT>::SharedPtr sub;

            sub = node_->create_subscription<MsgT>(
                topic_name,
                rclcpp::QoS(1),
                [promise](const typename MsgT::SharedPtr msg){
                    promise->set_value(*msg);
                }
            );
            if (future.wait_for(std::chrono::duration<double>(timeout_sec)) == std::future_status::ready){
                return future.get();
            }else {
                RCLCPP_ERROR(node_->get_logger(), "Timeout esperando en %s", topic_name.c_str());
                return MsgT(); // Devuelve un mensaje vacío si falla
            }
        }
  
 };
