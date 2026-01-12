#pragma once

#include <rclcpp/rclcpp.hpp>
#include "Ptp.hpp"
#include "cartesian.hpp"
#include "utils.hpp"

class OpenArmMove 
{
    private:
        rclcpp::Node::SharedPtr node_;
    
    public:
        OpenArmMove(rclcpp::Node::SharedPtr node);
        ~OpenArmMove();

        // Estrategias de movimiento
        Ptp *ptp;
        Cartesian *cartesian;
        Utils utils;
};

std::vector<geometry_msgs::msg::Pose> createpose(const std::vector<double>& pos, const std::vector<double>& rot);

