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


#define OPEN 0.044f
#define CLOSE 0.0f

class Ptp{
    public:
        Ptp(rclcpp::Node::SharedPtr node);
        ~Ptp();

        void PtpBimanual(const geometry_msgs::msg::Pose& target_left,
                         const geometry_msgs::msg::Pose& target_right,
                         float gripper_l, float gripper_r);
        void PtpBimanual(const geometry_msgs::msg::Pose& target_left,
                         std::string pose_right, float gripper_l,
                         float gripper_r);
        void PtpBimanual(std::string pose_left,
                         const geometry_msgs::msg::Pose& target_right,
                         float gripper_l, float gripper_r);
        void PtpBimanual(std::string pose_left, std::string pose_right);

        

        //void PtpBimanual(geometry_msgs::msg::Pose target_virtual, double gripper_left, double gripper_right);

        void PtpLeft(const geometry_msgs::msg::Pose& target, float gripper);
        void PtpLeft(std::string pose);
        void PtpRight(const geometry_msgs::msg::Pose& target, float gripper);
        void PtpRight(std::string pose);
       
        void PrintRobotInfo();
        
        void OpenLeftGripper();
        void CloseLeftGripper();
        void OpenRightGripper();
        void CloseRightGripper();
        
        geometry_msgs::msg::Pose get_target_pose_left();
        geometry_msgs::msg::Pose get_target_pose_right();
        geometry_msgs::msg::Pose get_current_pose_right();
        geometry_msgs::msg::Pose get_current_pose_left();
        geometry_msgs::msg::Pose get_current_dual_pose();
        
        geometry_msgs::msg::Pose PrevPose(
            const geometry_msgs::msg::Pose& Target_pos, float dist);

        void PrintPose(std::string pose_name, geometry_msgs::msg::Pose pose);
        
    private:
               double  MaxVelocityScalingFactor_;
        double  MaxAccelerationScalingFactor_;
        unsigned int     NumPlanningAttempts_;
        double  PlanningTime_;

        rclcpp::Node::SharedPtr node_;
     /*    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr tf_subscription_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_; */
       
        moveit::planning_interface::MoveGroupInterface* left_arm_;
        moveit::planning_interface::MoveGroupInterface* right_arm_;
        
        moveit::planning_interface::MoveGroupInterface* left_gripper_;
        moveit::planning_interface::MoveGroupInterface* right_gripper_;

        moveit::planning_interface::MoveGroupInterface* left_arm_full_;
        moveit::planning_interface::MoveGroupInterface* right_arm_full_;
        moveit::planning_interface::MoveGroupInterface* bimanual_;
        
        const moveit::core::JointModelGroup* left_joint_model_group_;
        const moveit::core::JointModelGroup* right_joint_model_group_;
        const moveit::core::JointModelGroup* bimanual_joint_model_group;
       

        geometry_msgs::msg::Pose target_pose_left;
        geometry_msgs::msg::Pose target_pose_right;


        moveit::planning_interface::MoveGroupInterface::Plan left_plan_;
        moveit::planning_interface::MoveGroupInterface::Plan right_plan_;
        moveit::planning_interface::MoveGroupInterface::Plan left_gripper_plan_;
        moveit::planning_interface::MoveGroupInterface::Plan right_gripper_plan_;
        moveit::planning_interface::MoveGroupInterface::Plan bimanual_plan_;

        Eigen::Isometry3d T_base_virtual_ ;
        rclcpp::TimerBase::SharedPtr timer_;
        void timerCallback();

        void setParam(moveit::planning_interface::MoveGroupInterface* group);
        void PlanArm(moveit::planning_interface::MoveGroupInterface* group,
                     const geometry_msgs::msg::Pose& target_pos,
                     std::string ee_link);
        
        void PlanGripper(moveit::planning_interface::MoveGroupInterface* group,
                     double gripper_pos);

        bool PlanLeftArm(const geometry_msgs::msg::Pose& target_pose);
        
        bool PlanRightArm(const geometry_msgs::msg::Pose& target_pose);
        bool PlanLeftFull(const geometry_msgs::msg::Pose& target_pose,
                           float gripper);
        bool PlanRightFull(const geometry_msgs::msg::Pose& target_pose,
                            float gripper);
        bool PlanBimanual(const geometry_msgs::msg::Pose& target_Left,
                           const geometry_msgs::msg::Pose& target_Right,
                           float gripper_left, float gripper_right);
        
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
        
        void LeftArmExec();
        void RightArmExec();
        bool BimanualExec();
        bool mergePlansWithInterpolation(
             const moveit::planning_interface::MoveGroupInterface::Plan&
                 left_plan,
             const moveit::planning_interface::MoveGroupInterface::Plan&
                 right_plan,
             moveit::planning_interface::MoveGroupInterface::Plan& out_plan,
             double right_delay_seconds, double min_dt);
        void CalculateOffset(geometry_msgs::msg::Pose target_virtual);
        geometry_msgs::msg::Pose EigenToPos(Eigen::Isometry3d eigen);
        void CalCurrentVirtual();
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_virtual_target_;
};