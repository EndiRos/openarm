// utils.hpp

#pragma once
#include "motion_strategy.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <vector>
#include <string>
#include "logger.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>  

class Utils{
    public:
        Utils(MotionStrategy* strategy);
        ~Utils();
        
        // Métodos de información del robot
        void PrintRobotInfo();
        
        // Métodos de obtención de poses actuales
        std::vector<geometry_msgs::msg::Pose> GetCurrentPoseLeft();
        std::vector<geometry_msgs::msg::Pose> GetCurrentPoseRight();
        
        // Utilidades de cálculo y visualización
        std::vector<geometry_msgs::msg::Pose> AproachPoint(const std::vector<geometry_msgs::msg::Pose> target_pos, float dist);
        void PrintPose(const std::string& pose_name, const geometry_msgs::msg::Pose& pose);
        std::vector<geometry_msgs::msg::Pose> TF2Vector(const tf2_msgs::msg::TFMessage& tf);
    
    private:
        // Conversión interna: TransformStamped -> vector<Pose> (1 elemento)
        MotionStrategy* strategy_;
};