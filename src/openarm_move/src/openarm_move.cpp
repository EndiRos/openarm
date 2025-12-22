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

#include "openarm_move.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include "logger.hpp"
#include "Topics_utils.hpp"


OpenArmMove::OpenArmMove(rclcpp::Node::SharedPtr node): node_(node) {
    
    MaxAccelerationScalingFactor_ = 0.1;
    MaxVelocityScalingFactor_ = 0.1;
    NumPlanningAttempts_ = 20;
    PlanningTime_= 5.0;
	left_arm_ = new moveit::planning_interface::MoveGroupInterface(node_,"left_arm");
    right_arm_ = new moveit::planning_interface::MoveGroupInterface(node_,"right_arm");
    left_arm_full_ = new moveit::planning_interface::MoveGroupInterface(node_,"left_arm_full");
    right_arm_full_ = new moveit::planning_interface::MoveGroupInterface(node_,"right_arm_full");
    bimanual_ =new moveit::planning_interface::MoveGroupInterface(node_,"bimanual_full");
    left_gripper_= new moveit::planning_interface::MoveGroupInterface(node_,"left_gripper");
    right_gripper_ = new moveit::planning_interface::MoveGroupInterface(node_,"right_gripper");
	
    left_arm_full_->startStateMonitor();
}

OpenArmMove::~OpenArmMove()
{
	if(left_arm_) delete left_arm_;
	if(right_arm_) delete right_arm_;
    if(left_gripper_) delete left_gripper_;
    if(right_gripper_) delete right_gripper_;
    if(left_arm_full_) delete left_arm_full_;
	if(right_arm_full_) delete right_arm_full_;
     if(bimanual_) delete bimanual_;
}

void OpenArmMove::CloseLeftGripper(){
    RCLCPP_INFO(LOGGER, "Closing left gripper");
    PlanGripper(left_gripper_, 0.0);
    left_gripper_->plan(left_gripper_plan_);
    left_gripper_->execute(left_gripper_plan_);
}

void OpenArmMove::CloseRightGripper(){
    RCLCPP_INFO(LOGGER, "Closing right gripper");
    PlanGripper(right_gripper_, 0.0);
    right_gripper_->plan(right_gripper_plan_);
    right_gripper_->execute(right_gripper_plan_);
}

void OpenArmMove::OpenLeftGripper()
{
    RCLCPP_INFO(LOGGER, "Opening left gripper");
    PlanGripper(left_gripper_, 0.04);
    left_gripper_->plan(left_gripper_plan_);
    left_gripper_->execute(left_gripper_plan_);
}

void OpenArmMove::OpenRightGripper(){
    RCLCPP_INFO(LOGGER, "Opening right gripper");
    PlanGripper(right_gripper_, 0.04);
    right_gripper_->plan(right_gripper_plan_);
    right_gripper_->execute(right_gripper_plan_);
}


void OpenArmMove::PtpBimanual(const geometry_msgs::msg::Pose& target_left,
                            const geometry_msgs::msg::Pose& target_right,
                            float gripper_l,
                            float gripper_r){
    RCLCPP_INFO(LOGGER, "Init PtpBimanual TARGET TARGET");
    PlanBimanual(target_left, target_right, gripper_l, gripper_r);
    BimanualExec();
}

void OpenArmMove::PtpBimanual(const geometry_msgs::msg::Pose& target_left,
                            std::string pose_right,
                            float gripper_l,
                            float gripper_r){
    
    RCLCPP_INFO(LOGGER, "Init PtpBimanual TARGET POSE");
    (void) gripper_r;
    PlanLeftFull(target_left, gripper_l);
    RightFullToNamedPose(pose_right);
    BimanualExec();
}

void OpenArmMove::PtpBimanual(std::string pose_left,
                            const geometry_msgs::msg::Pose& target_right,
                            float gripper_l,
                            float gripper_r){
    RCLCPP_INFO(LOGGER, "Init PtpBimanual POSE TARGET");
    (void) gripper_l;
    LeftFullToNamedPose(pose_left);
    PlanRightFull(target_right, gripper_r);
    BimanualExec();
}

void OpenArmMove::PtpBimanual(std::string pose_left,
                            std::string pose_right){
    RCLCPP_INFO(LOGGER, "Init PtpBimanual POSE POSE");
    BimanualNamedPose(pose_left, pose_right);
    BimanualExec();
}

void OpenArmMove::CalculateOffset(geometry_msgs::msg::Pose target_virtual){
    
    moveit::core::RobotStatePtr left_state = left_arm_full_->getCurrentState();
    moveit::core::RobotStatePtr right_state = right_arm_full_->getCurrentState();
    
    Eigen::Isometry3d T_base_left = left_state->getGlobalLinkTransform(left_arm_full_->getEndEffector());
    Eigen::Isometry3d T_base_right = right_state->getGlobalLinkTransform(right_arm_full_->getEndEffector());

    Eigen::Vector3d p_virtual =( T_base_left.translation() + T_base_right.translation()) * 0.5;
    Eigen::Quaterniond q_virtual = Eigen::Quaterniond(T_base_left.rotation()).slerp(0.5, Eigen::Quaterniond(T_base_right.rotation()));

    Eigen::Isometry3d T_base_virtual = Eigen::Isometry3d::Identity();
    T_base_virtual.translation()= p_virtual;
    T_base_virtual.linear() = q_virtual.toRotationMatrix();

    Eigen::Isometry3d offset_left = T_base_virtual.inverse() * T_base_left;
    Eigen::Isometry3d offset_right = T_base_right.inverse() * T_base_right;
}

void OpenArmMove::PtpBimanual(geometry_msgs::msg::Pose target_virtual){

    Eigen::Isometry3d offset_left, offset_right;
    CalculateOffset(target_virtual);

}

void OpenArmMove::PtpLeft(const geometry_msgs::msg::Pose &target, float gripper)
{
    RCLCPP_INFO(LOGGER, "Init Ptpleft TARGET");
    PlanLeftFull(target, gripper);
    LeftArmExec();
}

void OpenArmMove::PtpLeft(std::string pose)
{
    RCLCPP_INFO(LOGGER, "Init Ptpleft POSE");
    LeftFullToNamedPose(pose);
    LeftArmExec();
}

void OpenArmMove::PtpRight(const geometry_msgs::msg::Pose &target, float gripper)
{
    RCLCPP_INFO(LOGGER, "Init PtpRight TARGET");
    PlanRightFull(target, gripper);
    RightArmExec();
}

void OpenArmMove::PtpRight(std::string pose)
{
    RCLCPP_INFO(LOGGER, "Init PtpRight POSE");
    RightFullToNamedPose(pose);
    RightArmExec();
}