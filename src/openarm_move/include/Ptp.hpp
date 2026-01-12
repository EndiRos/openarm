/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE PtpODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Endika Etxebarrieta */

#pragma once
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2_ros/transform_listener.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <logger.hpp>
#include "motion_strategy.hpp"

#define OPEN 0.044f
#define CLOSE 0.0f

class Ptp : public MotionStrategy {
    public:
        Ptp(rclcpp::Node::SharedPtr node);
        ~Ptp() override;

        // --- Métodos de Alto Nivel (API para el usuario) ---
        // Estos llaman internamente a los métodos de planificación
        void Bimanual(const std::vector<geometry_msgs::msg::Pose>& target_left,
                         const std::vector<geometry_msgs::msg::Pose>& target_right,
                         float gripper_l, float gripper_r);
        void Bimanual(const std::vector<geometry_msgs::msg::Pose>& target_left,
                         std::string pose_right, float gripper_l,
                         float gripper_r);
        void Bimanual(std::string pose_left,
                         const std::vector<geometry_msgs::msg::Pose>& target_right,
                         float gripper_l, float gripper_r);
        void Bimanual(std::string pose_left, std::string pose_right);

        void Left(const std::vector<geometry_msgs::msg::Pose>& target, float gripper);
        void Left(std::string pose);
        void Right(const std::vector<geometry_msgs::msg::Pose>& target, float gripper);
        void Right(std::string pose);
       
        //void PrintRobotInfo();
        
        void OpenLeftGripper();
        void CloseLeftGripper();
        void OpenRightGripper();
        void CloseRightGripper();
        

    private:
        // --- Implementación de la interfaz MotionStrategy (Overrides) ---
        bool PlanLeftArm(const std::vector<geometry_msgs::msg::Pose>& waypoints) override;
        bool PlanRightArm(const std::vector<geometry_msgs::msg::Pose>& waypoints) override;
        
        bool PlanLeftArm_full(const std::vector<geometry_msgs::msg::Pose>& waypoints, double gripper_val) override;
        bool PlanRightArm_full(const std::vector<geometry_msgs::msg::Pose>& waypoints, double gripper_val) override;
        
        bool PlanBimanual(const std::vector<geometry_msgs::msg::Pose>& left_waypoints, 
                          const std::vector<geometry_msgs::msg::Pose>& right_waypoints, double left_gripper, double right_gripper) override;
        
        // Este método es específico de Ptp, se queda
        void PlanArm(moveit::planning_interface::MoveGroupInterface* group,
                     const geometry_msgs::msg::Pose& target_pos,
                     std::string ee_link);
        
        void PlanGripper(moveit::planning_interface::MoveGroupInterface* group,
                     double gripper_pos);

        // NOTA: He eliminado PlanLeftArm, PlanRightArm, PlanLeftFull, PlanRightFull de aquí
        // porque ya están declarados arriba como overrides públicos.
        
        bool PlanToNamedPose(
             moveit::planning_interface::MoveGroupInterface* group,
             moveit::planning_interface::MoveGroupInterface::Plan& plan_,
             const std::string pose_name);

        bool LeftToNamedPose(const std::string& pose_name);
        bool RightToNamedPose(const std::string& pose_name);
        bool LeftFullToNamedPose(const std::string& pose_name);
        bool PlanJointTarget(
            moveit::planning_interface::MoveGroupInterface* group,
            moveit::planning_interface::MoveGroupInterface::Plan& plan,
            const std::vector<double>& joint_values);
        bool RightFullToNamedPose(const std::string& pose_name);
        bool BimanualNamedPose(std::string left_pose, std::string right_pose);

        
        // Eliminado mergePlansWithInterpolation de aquí porque ya lo heredas de MotionStrategy
        
        void CalculateOffset(geometry_msgs::msg::Pose target_virtual);
        geometry_msgs::msg::Pose EigenToPos(Eigen::Isometry3d eigen);
        void CalCurrentVirtual();
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_virtual_target_;
};