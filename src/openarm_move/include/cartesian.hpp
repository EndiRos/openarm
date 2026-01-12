#pragma once
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <logger.hpp>
#include "motion_strategy.hpp"

class Cartesian : public MotionStrategy{
    public:
        Cartesian(rclcpp::Node::SharedPtr node); // Constructor corregido
        ~Cartesian() override;

        // Implementación correcta de la interfaz
        

    private :
        double eef_step_;
        double jump_thershold_;
        bool PlanLeftArm(const std::vector<geometry_msgs::msg::Pose>& waypoints) override;
        bool PlanRightArm(const std::vector<geometry_msgs::msg::Pose>& waypoints) override;
        
        bool PlanLeftArm_full(const std::vector<geometry_msgs::msg::Pose>& waypoints, double gripper_val) override;
        bool PlanRightArm_full(const std::vector<geometry_msgs::msg::Pose>& waypoints, double gripper_val) override;
        
        bool PlanBimanual(const std::vector<geometry_msgs::msg::Pose>& left_waypoints, 
                          const std::vector<geometry_msgs::msg::Pose>& right_waypoints, double left_grippe, double right_gripper) override;
        bool PlanArm(moveit::planning_interface::MoveGroupInterface* group,
                        moveit_msgs::msg::RobotTrajectory &trayectoty,
                        moveit::planning_interface::MoveGroupInterface::Plan &plan,
                        const std::vector<geometry_msgs::msg::Pose> &points);
        
        moveit_msgs::msg::RobotTrajectory left_trayectoty_;
        moveit_msgs::msg::RobotTrajectory right_trayectoty_;

};