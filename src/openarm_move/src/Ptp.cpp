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

#include "Ptp.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include "logger.hpp"
#include "Topics_utils.hpp"


Ptp::Ptp(rclcpp::Node::SharedPtr node): node_(node) {
    
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
    pub_virtual_target_= node_->create_publisher<geometry_msgs::msg::PoseStamped>("v_target", 10);
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Ptp::timerCallback, this)
    );
}

Ptp::~Ptp()
{
	if(left_arm_) delete left_arm_;
	if(right_arm_) delete right_arm_;
    if(left_gripper_) delete left_gripper_;
    if(right_gripper_) delete right_gripper_;
    if(left_arm_full_) delete left_arm_full_;
	if(right_arm_full_) delete right_arm_full_;
     if(bimanual_) delete bimanual_;
}

void Ptp::CloseLeftGripper(){
    RCLCPP_INFO(LOGGER, "Closing left gripper");
    PlanGripper(left_gripper_, 0.0);
    left_gripper_->plan(left_gripper_plan_);
    left_gripper_->execute(left_gripper_plan_);
}

void Ptp::CloseRightGripper(){
    RCLCPP_INFO(LOGGER, "Closing right gripper");
    PlanGripper(right_gripper_, 0.0);
    right_gripper_->plan(right_gripper_plan_);
    right_gripper_->execute(right_gripper_plan_);
}

void Ptp::OpenLeftGripper()
{
    RCLCPP_INFO(LOGGER, "Opening left gripper");
    PlanGripper(left_gripper_, 0.04);
    left_gripper_->plan(left_gripper_plan_);
    left_gripper_->execute(left_gripper_plan_);
}

void Ptp::OpenRightGripper(){
    RCLCPP_INFO(LOGGER, "Opening right gripper");
    PlanGripper(right_gripper_, 0.04);
    right_gripper_->plan(right_gripper_plan_);
    right_gripper_->execute(right_gripper_plan_);
}


void Ptp::PtpBimanual(const geometry_msgs::msg::Pose& target_left,
                            const geometry_msgs::msg::Pose& target_right,
                            float gripper_l,
                            float gripper_r){
    RCLCPP_INFO(LOGGER, "Init PtpBimanual TARGET TARGET");
    PlanBimanual(target_left, target_right, gripper_l, gripper_r);
    BimanualExec();
}

void Ptp::PtpBimanual(const geometry_msgs::msg::Pose& target_left,
                            std::string pose_right,
                            float gripper_l,
                            float gripper_r){
    
    RCLCPP_INFO(LOGGER, "Init PtpBimanual TARGET POSE");
    (void) gripper_r;
    PlanLeftFull(target_left, gripper_l);
    RightFullToNamedPose(pose_right);
    BimanualExec();
}

void Ptp::PtpBimanual(std::string pose_left,
                            const geometry_msgs::msg::Pose& target_right,
                            float gripper_l,
                            float gripper_r){
    RCLCPP_INFO(LOGGER, "Init PtpBimanual POSE TARGET");
    (void) gripper_l;
    LeftFullToNamedPose(pose_left);
    PlanRightFull(target_right, gripper_r);
    BimanualExec();
}

void Ptp::PtpBimanual(std::string pose_left,
                            std::string pose_right){
    RCLCPP_INFO(LOGGER, "Init PtpBimanual POSE POSE");
    BimanualNamedPose(pose_left, pose_right);
    BimanualExec();
}

/* void Ptp::PtpBimanual(geometry_msgs::msg::Pose target_virtual, double gripper_left, double gripper_right){
    
    moveit::core::RobotStatePtr left_state = left_arm_full_->getCurrentState();
    moveit::core::RobotStatePtr right_state = right_arm_full_->getCurrentState();
    
     // 1. Obtener transformaciones actuales de los end-effectors
    Eigen::Isometry3d T_base_left = left_state->getGlobalLinkTransform(left_arm_->getEndEffectorLink());
    Eigen::Isometry3d T_base_right = right_state->getGlobalLinkTransform(right_arm_->getEndEffectorLink());

    // 2. Calcular pose virtual actual (punto medio y orientación interpolada)
    CalCurrentVirtual();

    // 3. Calcular offsets relativos a la pose virtual actual
    // T_arm = T_virtual * T_offset  =>  T_offset = T_virtual^-1 * T_arm
    Eigen::Isometry3d offset_left = T_base_virtual_.inverse() * T_base_left;
    Eigen::Isometry3d offset_right = T_base_virtual_.inverse() * T_base_right;

    // 4. Convertir la pose virtual OBJETIVO (target_virtual) a Eigen
    Eigen::Isometry3d T_base_virtual_target = Eigen::Isometry3d::Identity();
    T_base_virtual_target.translation() = Eigen::Vector3d(target_virtual.position.x,
         target_virtual.position.y, target_virtual.position.z);
    Eigen::Quaterniond q_target (target_virtual.orientation.w, target_virtual.orientation.x, target_virtual.orientation.y,
        target_virtual.orientation.z); 
    T_base_virtual_target.linear()= q_target.toRotationMatrix();

    // 5. Calcular nuevas poses objetivo para los brazos aplicando los offsets
    Eigen::Isometry3d T_target_left = T_base_virtual_target * offset_left;
    Eigen::Isometry3d T_target_right = T_base_virtual_target * offset_right;

    geometry_msgs::msg::Pose target_pose_left = EigenToPos(T_target_left);
    geometry_msgs::msg::Pose target_pose_right = EigenToPos(T_target_right);

    PtpBimanual(target_pose_left, target_pose_right, gripper_left, gripper_right);
}
 */



void Ptp::PtpLeft(const geometry_msgs::msg::Pose &target, float gripper)
{
    RCLCPP_INFO(LOGGER, "Init Ptpleft TARGET");
    PlanLeftFull(target, gripper);
    LeftArmExec();
}

void Ptp::PtpLeft(std::string pose)
{
    RCLCPP_INFO(LOGGER, "Init Ptpleft POSE");
    LeftFullToNamedPose(pose);
    LeftArmExec();
}

void Ptp::PtpRight(const geometry_msgs::msg::Pose &target, float gripper)
{
    RCLCPP_INFO(LOGGER, "Init PtpRight TARGET");
    PlanRightFull(target, gripper);
    RightArmExec();
}

void Ptp::PtpRight(std::string pose)
{
    RCLCPP_INFO(LOGGER, "Init PtpRight POSE");
    RightFullToNamedPose(pose);
    RightArmExec();
}