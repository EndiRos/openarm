#include "motion_strategy.hpp"
#include <cmath>
#include <algorithm>
#include <map>
#include <set>

// Funciones auxiliares (helpers) locales al archivo
static double duration_to_seconds(const builtin_interfaces::msg::Duration &d)
{
    return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}

static builtin_interfaces::msg::Duration seconds_to_duration(double t)
{
    builtin_interfaces::msg::Duration d;
    d.sec = static_cast<int32_t>(std::floor(t));
    d.nanosec = static_cast<uint32_t>((t - std::floor(t)) * 1e9);
    return d;
}

static double sample_joint_from_traj(const trajectory_msgs::msg::JointTrajectory &traj,
                                     const std::string &joint_name, double t)
{
    auto it = std::find(traj.joint_names.begin(), traj.joint_names.end(), joint_name);
    if (it == traj.joint_names.end()) return std::numeric_limits<double>::quiet_NaN();
    size_t jidx = std::distance(traj.joint_names.begin(), it);
    const auto &pts = traj.points;
    if (pts.empty()) return std::numeric_limits<double>::quiet_NaN();
    if (pts.size() == 1) {
        if (jidx < pts[0].positions.size()) return pts[0].positions[jidx];
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    size_t idx0 = 0;
    while (idx0 + 1 < pts.size() && duration_to_seconds(pts[idx0 + 1].time_from_start) < t) ++idx0;
    
    if (idx0 + 1 >= pts.size()) {
        const auto &p = pts.back();
        if (jidx < p.positions.size()) return p.positions[jidx];
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    const auto &p0 = pts[idx0];
    const auto &p1 = pts[idx0 + 1];
    double t0 = duration_to_seconds(p0.time_from_start);
    double t1 = duration_to_seconds(p1.time_from_start);
    
    if (t1 <= t0) {
        if (jidx < p1.positions.size()) return p1.positions[jidx];
        if (jidx < p0.positions.size()) return p0.positions[jidx];
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    double alpha = (t - t0) / (t1 - t0);
    double v0 = (jidx < p0.positions.size()) ? p0.positions[jidx] : 0.0;
    double v1 = (jidx < p1.positions.size()) ? p1.positions[jidx] : v0;
    return v0 + alpha * (v1 - v0);
}

MotionStrategy::MotionStrategy(rclcpp::Node::SharedPtr node) : node_ (node){
    MaxAccelerationScalingFactor_ = 0.1;
    MaxVelocityScalingFactor_ = 0.1;
    NumPlanningAttempts_ = 20;
    PlanningTime_= 5.0;
    
    left_arm_ = new moveit::planning_interface::MoveGroupInterface(node_,"left_arm");
    right_arm_ = new moveit::planning_interface::MoveGroupInterface(node_,"right_arm");
    left_arm_full_ = new moveit::planning_interface::MoveGroupInterface(node_,"left_arm_full");
    right_arm_full_ = new moveit::planning_interface::MoveGroupInterface(node_,"right_arm_full");
    bimanual_ = new moveit::planning_interface::MoveGroupInterface(node_,"bimanual_full");
    left_gripper_= new moveit::planning_interface::MoveGroupInterface(node_,"left_gripper");
    right_gripper_ = new moveit::planning_interface::MoveGroupInterface(node_,"right_gripper");
    
    // Es recomendable iniciar el monitor de estado
    left_arm_full_->startStateMonitor();
    right_arm_full_->startStateMonitor();
    bimanual_->startStateMonitor();
}

MotionStrategy::~MotionStrategy() {
    if(left_arm_) delete left_arm_;
    if(right_arm_) delete right_arm_;
    if(left_gripper_) delete left_gripper_;
    if(right_gripper_) delete right_gripper_;
    if(left_arm_full_) delete left_arm_full_;
    if(right_arm_full_) delete right_arm_full_;
    if(bimanual_) delete bimanual_;
}

bool MotionStrategy::ExecuteLeft()
{
    try {
        robot_trajectory::RobotTrajectory rt(left_arm_full_->getCurrentState()->getRobotModel(), left_arm_full_->getName());
        rt.setRobotTrajectoryMsg(*left_arm_full_->getCurrentState(), left_plan_.trajectory_);
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        if (iptp.computeTimeStamps(rt)) {
            rt.getRobotTrajectoryMsg(left_plan_.trajectory_);
        } else {
            RCLCPP_WARN(node_->get_logger(), "Time parameterization failed for left_plan_");
            return false;
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(node_->get_logger(), "Exception during left trajectory retiming: %s", e.what());
        return false;
    }
    return (left_arm_full_->execute(left_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
}

bool MotionStrategy::ExecuteRight()
{
    try {
        robot_trajectory::RobotTrajectory rt(right_arm_full_->getCurrentState()->getRobotModel(), right_arm_full_->getName());
        rt.setRobotTrajectoryMsg(*right_arm_full_->getCurrentState(), right_plan_.trajectory_);
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        if (iptp.computeTimeStamps(rt)) {
            rt.getRobotTrajectoryMsg(right_plan_.trajectory_);
        } else {
            RCLCPP_WARN(node_->get_logger(), "Time parameterization failed for right_plan_");
            return false;
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(node_->get_logger(), "Exception during right trajectory retiming: %s", e.what());
        return false;
    }

    return (right_arm_full_->execute(right_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
}

bool MotionStrategy::ExecuteBimanual()
{
    bool ok_merge = mergePlansWithInterpolation(0.0, 0.02);
    if (!ok_merge) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to merge left/right plans into bimanual plan");
        return false;
    }

    try {
        robot_trajectory::RobotTrajectory rt(bimanual_->getCurrentState()->getRobotModel(), bimanual_->getName());
        rt.setRobotTrajectoryMsg(*bimanual_->getCurrentState(), bimanual_plan_.trajectory_);
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        if (iptp.computeTimeStamps(rt)) {
            rt.getRobotTrajectoryMsg(bimanual_plan_.trajectory_);
        } else {
            RCLCPP_WARN(node_->get_logger(), "Time parameterization failed for bimanual_plan_");
            return false;
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(node_->get_logger(), "Exception during bimanual trajectory retiming: %s", e.what());
        return false;
    }

    // Forzar que el primer punto del plan bimanual coincida con el estado actual
    try {
        auto rs = bimanual_->getCurrentState();
        if (rs) {
            const moveit::core::JointModelGroup *jmg = rs->getJointModelGroup("bimanual_full");
            if (jmg && !bimanual_plan_.trajectory_.joint_trajectory.points.empty()) {
                std::vector<double> cur_positions;
                cur_positions.resize(jmg->getVariableCount());
                rs->copyJointGroupPositions(jmg, cur_positions);
                auto &pt0 = bimanual_plan_.trajectory_.joint_trajectory.points[0];
                
                // Lógica de mapeo simplificada para el ejemplo
                if (pt0.positions.size() == cur_positions.size()) {
                    pt0.positions = cur_positions;
                    pt0.velocities.assign(cur_positions.size(), 0.0);
                    pt0.effort.clear();
                    pt0.time_from_start.sec = 0;
                    pt0.time_from_start.nanosec = 0;
                }
            }
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(node_->get_logger(), "Exception forcing bimanual start point: %s", e.what());
        return false;
    }

    return (bimanual_->execute(bimanual_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
}

// NOTA: Quitamos los valores por defecto (= 0.0) aquí en la implementación
bool MotionStrategy::mergePlansWithInterpolation(double right_delay_seconds, double min_dt) 
{
    const std::vector<std::string> target_joints = bimanual_->getJointNames();
    if (target_joints.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "merge: bimanual group reports no joints");
        return false;
    }

    const auto &left_traj = left_plan_.trajectory_.joint_trajectory;
    const auto &right_traj = right_plan_.trajectory_.joint_trajectory;

    std::set<double> times_set;
    for (const auto &p : left_traj.points) times_set.insert(duration_to_seconds(p.time_from_start));
    for (const auto &p : right_traj.points) times_set.insert(duration_to_seconds(p.time_from_start) + right_delay_seconds);

    if (times_set.empty()) {
        times_set.insert(0.0);
    }

    std::vector<double> times;
    double last_t = -1e9;
    for (double t : times_set) {
        if (last_t < -1e8) {
            times.push_back(t);
            last_t = t;
        } else {
            if (t - last_t < min_dt) {
                last_t += min_dt;
                times.push_back(last_t);
            } else {
                times.push_back(t);
                last_t = t;
            }
        }
    }

    // Obtener estado actual para valores por defecto
    moveit::core::RobotStatePtr cur = bimanual_->getCurrentState();
    std::vector<double> cur_vals(target_joints.size(), 0.0);
    if (cur) {
        // copiar posiciones actuales en el orden target_joints
        for (size_t i = 0; i < target_joints.size(); ++i) {
            const std::string &jn = target_joints[i];
            const moveit::core::JointModel *jm = cur->getJointModel(jn);
            if (jm && jm->getVariableCount() == 1) {
                std::vector<double> tmp;
                cur->copyJointGroupPositions(cur->getRobotModel()->getJointModelGroup(bimanual_->getName()), tmp);
                // fallback: intentamos localizar índice — pero para simplicidad dejamos cur_vals[i] = 0 si no encontramos
            }
        }
        // Nota: para seguridad vamos a pedir posiciones por grupo completo:
        std::vector<double> all;
        // intentar copiar por nombre de grupo bimanual
        try {
            cur->copyJointGroupPositions(bimanual_->getName(), all);
            // all está en el orden del JointModelGroup; necesitamos mapearlo a target_joints
            const moveit::core::JointModelGroup *jmg = cur->getJointModelGroup(bimanual_->getName());
            if (jmg) {
                std::vector<std::string> jmg_names = jmg->getVariableNames();
                for (size_t i = 0; i < target_joints.size(); ++i) {
                    const std::string &jn = target_joints[i];
                    auto it = std::find(jmg_names.begin(), jmg_names.end(), jn);
                    if (it != jmg_names.end()) {
                        size_t idx = std::distance(jmg_names.begin(), it);
                        if (idx < all.size()) cur_vals[i] = all[idx];
                    }
                }
            }
        } catch (...) {
            // ignorar, ya tenemos cur_vals=0
        }
    }

    trajectory_msgs::msg::JointTrajectory merged;
    merged.joint_names = target_joints;
    merged.points.clear();

    for (double t : times) {
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions.resize(target_joints.size());
        pt.velocities.clear();
        pt.accelerations.clear();
        pt.effort.clear();
        pt.time_from_start = seconds_to_duration(t);

        for (size_t i = 0; i < target_joints.size(); ++i) {
            const std::string &jn = target_joints[i];
            double v_left = sample_joint_from_traj(left_traj, jn, t);
            double v_right = sample_joint_from_traj(right_traj, jn, t - right_delay_seconds);

            double v = std::numeric_limits<double>::quiet_NaN();
            if (!std::isnan(v_left)) v = v_left;
            if (!std::isnan(v_right)) {
                if (std::isnan(v)) v = v_right;
                else v = v_left; // Prioridad izquierda si conflicto
            }
            if (std::isnan(v)) v = cur_vals[i]; // Fallback a valor actual
            pt.positions[i] = v;
        }
        merged.points.push_back(pt);
    }

    bimanual_plan_.trajectory_.joint_trajectory = merged;
    RCLCPP_INFO(node_->get_logger(), "merge: created merged trajectory with %zu points", merged.points.size());
    return true;
}

void MotionStrategy::setParam(moveit::planning_interface::MoveGroupInterface* group) {
    if (!group) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Group es nulo en setParam");
        return;
    }
    
    // Set planning time and attempts
    group->setPlanningTime(PlanningTime_);
    group->setNumPlanningAttempts(NumPlanningAttempts_);
    
    // Set planner ID
    group->setPlannerId("RRTstar");
    
    // Set scaling factors
    group->setMaxVelocityScalingFactor(MaxVelocityScalingFactor_);
    group->setMaxAccelerationScalingFactor(MaxAccelerationScalingFactor_);
    
    // Set goal tolerance
    group->setGoalPositionTolerance(0.01);      // 1 cm
    group->setGoalOrientationTolerance(0.01);   // ~0.57 degrees
    group->setGoalJointTolerance(0.01);
    
    RCLCPP_DEBUG(node_->get_logger(), "✓ Parámetros de planificación configurados para grupo: %s", 
                 group->getName().c_str());
}

