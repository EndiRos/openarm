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


void OpenArmMove::LeftArmExec()
{
    // reparametrizar en el tiempo la trayectoria para evitar puntos con time_from_start duplicado
    try {
        robot_trajectory::RobotTrajectory rt(left_arm_full_->getCurrentState()->getRobotModel(), left_arm_full_->getName());
        rt.setRobotTrajectoryMsg(*left_arm_full_->getCurrentState(), left_plan_.trajectory_);
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        if (iptp.computeTimeStamps(rt)) {
            rt.getRobotTrajectoryMsg(left_plan_.trajectory_);
        } else {
            RCLCPP_WARN(LOGGER, "Time parameterization failed for left_plan_");
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(LOGGER, "Exception during left trajectory retiming: %s", e.what());
    }
    left_arm_full_->execute(left_plan_);
}

void OpenArmMove::RightArmExec()
{
    try {
        robot_trajectory::RobotTrajectory rt(right_arm_full_->getCurrentState()->getRobotModel(), right_arm_full_->getName());
        rt.setRobotTrajectoryMsg(*right_arm_full_->getCurrentState(), right_plan_.trajectory_);
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        if (iptp.computeTimeStamps(rt)) {
            rt.getRobotTrajectoryMsg(right_plan_.trajectory_);
        } else {
            RCLCPP_WARN(LOGGER, "Time parameterization failed for right_plan_");
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(LOGGER, "Exception during right trajectory retiming: %s", e.what());
    }
    right_arm_full_->execute(right_plan_);
}

bool OpenArmMove::BimanualExec()
{
    bool ok_merge = mergePlansWithInterpolation(left_plan_, right_plan_, bimanual_plan_, /*right_delay_seconds=*/0.0, /*min_dt=*/0.02);
    if (!ok_merge) {
        RCLCPP_ERROR(LOGGER, "Failed to merge left/right plans into bimanual plan");
        return false;
    }

    try {
        robot_trajectory::RobotTrajectory rt(bimanual_->getCurrentState()->getRobotModel(), bimanual_->getName());
        rt.setRobotTrajectoryMsg(*bimanual_->getCurrentState(), bimanual_plan_.trajectory_);
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        if (iptp.computeTimeStamps(rt)) {
            rt.getRobotTrajectoryMsg(bimanual_plan_.trajectory_);
        } else {
            RCLCPP_WARN(LOGGER, "Time parameterization failed for bimanual_plan_");
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(LOGGER, "Exception during bimanual trajectory retiming: %s", e.what());
    }

    // Forzar que el primer punto del plan bimanual coincida con el estado actual del grupo bimanual
    try {
        auto rs = bimanual_->getCurrentState();
        if (rs) {
            const moveit::core::JointModelGroup *jmg = rs->getJointModelGroup("bimanual_full");
            if (jmg && !bimanual_plan_.trajectory_.joint_trajectory.points.empty()) {
                std::vector<double> cur_positions;
                cur_positions.resize(jmg->getVariableCount());
                rs->copyJointGroupPositions(jmg, cur_positions);
                auto &pt0 = bimanual_plan_.trajectory_.joint_trajectory.points[0];
                if (pt0.positions.size() == cur_positions.size()) {
                    pt0.positions = cur_positions;
                    pt0.velocities.assign(cur_positions.size(), 0.0);
                    pt0.effort.clear();
                    pt0.time_from_start.sec = 0;
                    pt0.time_from_start.nanosec = 0;
                } else {
                    // Si el tamaño no coincide, intenta mapear por nombre (fallback)
                    const auto &names = bimanual_plan_.trajectory_.joint_trajectory.joint_names;
                    if (names.size() == cur_positions.size() || !names.empty()) {
                        std::vector<double> mapped(cur_positions.size(), 0.0);
                        // obtener un RobotState con positions por nombre:
                        std::map<std::string,double> cur_map;
                        std::vector<std::string> jmg_names = jmg->getVariableNames();
                        for (size_t i = 0; i < jmg_names.size() && i < cur_positions.size(); ++i) cur_map[jmg_names[i]] = cur_positions[i];
                        for (size_t i = 0; i < names.size(); ++i) {
                            auto it = cur_map.find(names[i]);
                            if (it != cur_map.end()) mapped[i] = it->second;
                            else mapped[i] = pt0.positions.size() > i ? pt0.positions[i] : 0.0;
                        }
                        pt0.positions = mapped;
                        pt0.velocities.assign(mapped.size(), 0.0);
                        pt0.effort.clear();
                        pt0.time_from_start.sec = 0;
                        pt0.time_from_start.nanosec = 0;
                    }
                }
            }
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(LOGGER, "Exception forcing bimanual start point: %s", e.what());
    }

    bimanual_->execute(bimanual_plan_);
    return true;
}


