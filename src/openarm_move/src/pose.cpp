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
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Endika Etxebarrieta */

#include "openarm_move.hpp"

bool Ptp::BimanualNamedPose(std::string left_pose, std::string right_pose)
{
    // --- BRAZO IZQUIERDO ---
    setParam(left_arm_full_);
    
    // Dar un poco de tolerancia al objetivo por si hay pequeñas colisiones o errores numéricos
    left_arm_full_->setGoalJointTolerance(0.01); 

    left_arm_full_->setStartStateToCurrentState();

    if (!left_arm_full_->setNamedTarget(left_pose)) {
        RCLCPP_ERROR(LOGGER, "Named pose '%s' not found for left_arm_full", left_pose.c_str());
        return false;
    }

    moveit::core::MoveItErrorCode left_result = left_arm_full_->plan(left_plan_);
    bool left_ok = (left_result == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (!left_ok) {
        RCLCPP_ERROR(LOGGER, "Planning left_arm_full named pose failed. Error Code: %d", left_result.val);
        // Si el error es -1 (FAILURE) o -5 (TIMED_OUT), no encuentra camino.
        // Si es -4 (INVALID_GOAL_STATE), el destino choca.
    }

    // --- BRAZO DERECHO ---
    setParam(right_arm_full_);

    right_arm_full_->setStartStateToCurrentState();

    if (!right_arm_full_->setNamedTarget(right_pose)) {
        RCLCPP_ERROR(LOGGER, "Named pose '%s' not found for right_arm_full", right_pose.c_str());
        return false;
    }

    moveit::core::MoveItErrorCode right_result = right_arm_full_->plan(right_plan_);
    bool right_ok = (right_result == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (!right_ok) {
        RCLCPP_ERROR(LOGGER, "Planning right_arm_full named pose failed. Error Code: %d", right_result.val);
        // Si el error es -1 (FAILURE) o -5 (TIMED_OUT), no encuentra camino.
        // Si es -4 (INVALID_GOAL_STATE), el destino choca.
    }

    return left_ok && right_ok;

}

bool Ptp::PlanToNamedPose(moveit::planning_interface::MoveGroupInterface *group,
    moveit::planning_interface::MoveGroupInterface::Plan &my_plan,
    const std::string pose_name)
{
    if (!group) {
        RCLCPP_ERROR(LOGGER, "MoveGroup not initilalized");
        return false;
    }

    RCLCPP_INFO(LOGGER, "Go to pose named: %s", pose_name.c_str());
    
    setParam(group);
    group->setStartStateToCurrentState();
    
    group->setNamedTarget(pose_name);

    moveit::core::MoveItErrorCode ok = group->plan(my_plan);
    if (ok == moveit::core::MoveItErrorCode::SUCCESS) {
        return true;
    } else {
        RCLCPP_ERROR(LOGGER, "Failure planing poss named '%s' (code=%d)", pose_name.c_str(), ok.val);
        return false;
    };
}

bool Ptp::LeftToNamedPose(const std::string &pose_name)
{
    return PlanToNamedPose(left_arm_, left_plan_ ,pose_name);
}

bool Ptp::RightToNamedPose(const std::string &pose_name)
{
    return PlanToNamedPose(right_arm_, right_plan_, pose_name);
}

bool Ptp::RightFullToNamedPose(const std::string &pose_name)
{
    return PlanToNamedPose(right_arm_full_, right_plan_, pose_name);
}

bool Ptp::LeftFullToNamedPose(const std::string &pose_name)
{
    return PlanToNamedPose(left_arm_full_, left_plan_,  pose_name);
}
