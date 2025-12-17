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
    PlanningTime_= 20;
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
    std::vector<double> gripper_joint_positions(1);
    gripper_joint_positions[0] = 0.00;  // Posición cerrada
    left_gripper_->setJointValueTarget(gripper_joint_positions);
}

void OpenArmMove::CloseRightGripper(){
    RCLCPP_INFO(LOGGER, "Closing right gripper");
    std::vector<double> gripper_joint_positions(1);
    gripper_joint_positions[0] = 0.0;  // Posición cerrada
    right_gripper_->setJointValueTarget(gripper_joint_positions);
}

void OpenArmMove::OpenLeftGripper()
{
    RCLCPP_INFO(LOGGER, "Opening left gripper");
    std::vector<double> gripper_joint_positions(1);
    gripper_joint_positions[0] = 0.04;  // Posición cerrada
    left_gripper_->setJointValueTarget(gripper_joint_positions);
}

void OpenArmMove::OpenRightGripper(){
    RCLCPP_INFO(LOGGER, "Opening right gripper");
    std::vector<double> gripper_joint_positions(1);
    gripper_joint_positions[0] = 0.04;  // Posición cerrada
    right_gripper_->setJointValueTarget(gripper_joint_positions);
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