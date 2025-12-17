#include <set>
#include <limits>
#include <openarm_move.hpp>


// helper: convierte Duration -> segundos (double)
static double duration_to_seconds(const builtin_interfaces::msg::Duration &d)
{
    return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}

// helper: crea Duration desde segundos
static builtin_interfaces::msg::Duration seconds_to_duration(double t)
{
    builtin_interfaces::msg::Duration d;
    d.sec = static_cast<int32_t>(std::floor(t));
    d.nanosec = static_cast<uint32_t>((t - std::floor(t)) * 1e9);
    return d;
}

// helper: samplea/interpola una joint dentro de una JointTrajectory en tiempo t (seconds).
// Si la joint no existe en la trayectoria devuelve NaN.
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
    // localizar intervalo
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

// Función principal: fusiona left_plan y right_plan en out_plan; right_delay permite retrasar inicio del derecho.
bool OpenArmMove::mergePlansWithInterpolation(
    const moveit::planning_interface::MoveGroupInterface::Plan &left_plan,
    const moveit::planning_interface::MoveGroupInterface::Plan &right_plan,
    moveit::planning_interface::MoveGroupInterface::Plan &out_plan,
    double right_delay_seconds = 0.0,
    double min_dt = 0.02) // tiempo mínimo entre puntos
{
    // 1) joints objetivo en el orden que espera el controlador
    const std::vector<std::string> target_joints = bimanual_->getJointNames();
    if (target_joints.empty()) {
        RCLCPP_ERROR(LOGGER, "merge: bimanual group reports no joints");
        return false;
    }

    const auto &left_traj = left_plan.trajectory_.joint_trajectory;
    const auto &right_traj = right_plan.trajectory_.joint_trajectory;

    // 2) recolectar instantes (union) como doubles (seconds)
    std::set<double> times_set;
    for (const auto &p : left_traj.points) times_set.insert(duration_to_seconds(p.time_from_start));
    for (const auto &p : right_traj.points) times_set.insert(duration_to_seconds(p.time_from_start) + right_delay_seconds);

    if (times_set.empty()) {
        RCLCPP_WARN(LOGGER, "merge: no time points found in subplans; creating single-point using current state");
        // crear single time=0
        times_set.insert(0.0);
    }

    // opcional: asegurarnos de distancia mínima entre tiempos (evitar duplicados exactos)
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

    // 3) obtener estado actual para valores por defecto
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

    // 4) construir merged JointTrajectory
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
                // si ambos tienen valor y son de distinto brazo, decide prioridad (normalmente no se solapan)
                if (std::isnan(v)) v = v_right;
                else {
                    // si aparecen en ambos, dejamos v_left (o podrías mezclar)
                    v = v_left;
                }
            }
            if (std::isnan(v)) v = cur_vals[i];
            pt.positions[i] = v;
        }
        merged.points.push_back(pt);
    }

    // 5) empaquetar en out_plan
    out_plan = moveit::planning_interface::MoveGroupInterface::Plan();
    out_plan.trajectory_.joint_trajectory = merged;

    RCLCPP_INFO(LOGGER, "merge: created merged trajectory with %zu points and %zu joints",
                merged.points.size(), merged.joint_names.size());
    return true;
}