#include <openarm_move.hpp>

void Ptp::PrintRobotInfo(){
    RCLCPP_INFO(LOGGER, "=== LEFT ARM ===");
    RCLCPP_INFO(LOGGER, "Planning frame: %s", left_arm_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", left_arm_->getEndEffectorLink().c_str());
    
    geometry_msgs::msg::PoseStamped left_pose = left_arm_->getCurrentPose();
    RCLCPP_INFO(LOGGER, "Position: x:%f y:%f z:%f",
        left_pose.pose.position.x, left_pose.pose.position.y, left_pose.pose.position.z);
    RCLCPP_INFO(LOGGER, "Orientation (Quaternión) x:%f, y:%f, z:%f, w:%f",
        left_pose.pose.orientation.x,
        left_pose.pose.orientation.y,
        left_pose.pose.orientation.z,
        left_pose.pose.orientation.w);
    
    // Convertir quaternión a Euler
    tf2::Quaternion q_left(
        left_pose.pose.orientation.x,
        left_pose.pose.orientation.y,
        left_pose.pose.orientation.z,
        left_pose.pose.orientation.w
    );
    tf2::Matrix3x3 m_left(q_left);
    double roll_left, pitch_left, yaw_left;
    m_left.getRPY(roll_left, pitch_left, yaw_left);
    
    RCLCPP_INFO(LOGGER, "Orientation (Euler) Roll:%f Pitch:%f Yaw:%f (radianes)",
        roll_left, pitch_left, yaw_left);
    RCLCPP_INFO(LOGGER, "Orientation (Euler) Roll:%f° Pitch:%f° Yaw:%f° (grados)",
        roll_left * 180.0 / M_PI, pitch_left * 180.0 / M_PI, yaw_left * 180.0 / M_PI);
    
    RCLCPP_INFO(LOGGER, "=== RIGHT ARM ===");
    RCLCPP_INFO(LOGGER, "Planning frame: %s", right_arm_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", right_arm_->getEndEffectorLink().c_str());
    
    geometry_msgs::msg::PoseStamped right_pose = right_arm_->getCurrentPose();
    RCLCPP_INFO(LOGGER, "Position: x:%f y:%f z:%f",
        right_pose.pose.position.x, right_pose.pose.position.y, right_pose.pose.position.z);
    RCLCPP_INFO(LOGGER, "Orientation (Quaternión) x:%f, y:%f, z:%f, w:%f",
        right_pose.pose.orientation.x,
        right_pose.pose.orientation.y,
        right_pose.pose.orientation.z,
        right_pose.pose.orientation.w);
    
    // Convertir quaternión a Euler
    tf2::Quaternion q_right(
        right_pose.pose.orientation.x,
        right_pose.pose.orientation.y,
        right_pose.pose.orientation.z,
        right_pose.pose.orientation.w
    );
    tf2::Matrix3x3 m_right(q_right);
    double roll_right, pitch_right, yaw_right;
    m_right.getRPY(roll_right, pitch_right, yaw_right);
    
    RCLCPP_INFO(LOGGER, "Orientation (Euler) Roll:%f Pitch:%f Yaw:%f (radianes)",
        roll_right, pitch_right, yaw_right);
    RCLCPP_INFO(LOGGER, "Orientation (Euler) Roll:%f° Pitch:%f° Yaw:%f° (grados)",
        roll_right * 180.0 / M_PI, pitch_right * 180.0 / M_PI, yaw_right * 180.0 / M_PI);
}

geometry_msgs::msg::Pose Ptp::get_target_pose_left(){
    return target_pose_left;
}

geometry_msgs::msg::Pose Ptp::get_current_pose_right(){
    geometry_msgs::msg::Pose empty_pose;
    if (!right_arm_full_) {
        RCLCPP_ERROR(LOGGER, "right_arm_full_ no inicializado en get_current_pose_right()");
        return empty_pose;
    }
    // getCurrentPose() devuelve PoseStamped -> devolver solo .pose
    try {
        geometry_msgs::msg::PoseStamped ps = right_arm_full_->getCurrentPose();
        return ps.pose;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "Excepción al obtener pose actual del brazo derecho: %s", e.what());
        return empty_pose;
    }
}

geometry_msgs::msg::Pose Ptp::get_current_pose_left(){
    geometry_msgs::msg::Pose empty_pose;
    if (!left_arm_full_) {
        RCLCPP_ERROR(LOGGER, "right_arm_full_ no inicializado en get_current_pose_right()");
        return empty_pose;
    }
    // getCurrentPose() devuelve PoseStamped -> devolver solo .pose
    try {
        geometry_msgs::msg::PoseStamped ps = left_arm_full_->getCurrentPose();
        return ps.pose;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "Excepción al obtener pose actual del brazo derecho: %s", e.what());
        return empty_pose;
    }
}

geometry_msgs::msg::Pose Ptp::get_current_dual_pose(){
    CalCurrentVirtual();
    return EigenToPos(T_base_virtual_);
}
geometry_msgs::msg::Pose Ptp::PrevPose(const geometry_msgs::msg::Pose &Target_pos, float dist)
{
    geometry_msgs::msg::Pose target;
    target = Target_pos;
    tf2::Quaternion q;
    q.setX(Target_pos.orientation.x);
    q.setY(Target_pos.orientation.y);
    q.setZ(Target_pos.orientation.z);
    q.setW(Target_pos.orientation.w);

    tf2::Matrix3x3 m(q);
    tf2::Vector3 z_axis = m.getColumn(2);
    target.position.x = Target_pos.position.x - z_axis.x() * dist;
    target.position.y = Target_pos.position.y - z_axis.y() * dist;
    target.position.z = Target_pos.position.z - z_axis.z() * dist;

    return target;

}

void Ptp::timerCallback() {
    
    CalCurrentVirtual();
    if (pub_virtual_target_){
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = node_->now();
        msg.header.frame_id = left_arm_full_->getPlanningFrame();
        msg.pose = EigenToPos(T_base_virtual_);
        pub_virtual_target_->publish(msg);
    }
}

void Ptp::setParam(
    moveit::planning_interface::MoveGroupInterface* group) {
  group->setMaxVelocityScalingFactor(MaxVelocityScalingFactor_);
  group->setMaxAccelerationScalingFactor(MaxAccelerationScalingFactor_);
  group->setNumPlanningAttempts(NumPlanningAttempts_);
  group->setPlanningTime(PlanningTime_);
  group->setGoalJointTolerance(0.01);
}

void Ptp::PrintPose(std::string pose_name, geometry_msgs::msg::Pose pose)
{
    RCLCPP_INFO(LOGGER, "Pose name: %s", pose_name.c_str());
    RCLCPP_INFO(LOGGER, "Position: x:%f y:%f z:%f",
        pose.position.x, pose.position.y, pose.position.z);
    RCLCPP_INFO(LOGGER, "Orientation (Quaternion) x:%f, y:%f, z:%f, w:%f",
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
}

geometry_msgs::msg::Pose Ptp::EigenToPos(Eigen::Isometry3d eigen)
{
    geometry_msgs::msg::Pose ret;
    ret.position.x = eigen.translation().x();
    ret.position.y = eigen.translation().y();
    ret.position.z = eigen.translation().z();
    Eigen::Quaterniond q_tl(eigen.rotation());
    ret.orientation.x = q_tl.x();
    ret.orientation.y = q_tl.y();
    ret.orientation.z = q_tl.z();
    ret.orientation.w = q_tl.w();

    return ret;
}

void Ptp::CalCurrentVirtual(){

    moveit::core::RobotStatePtr left_state = left_arm_full_->getCurrentState();
    moveit::core::RobotStatePtr right_state = right_arm_full_->getCurrentState();
     // 1. Obtener transformaciones actuales de los end-effectors
    Eigen::Isometry3d T_base_left = left_state->getGlobalLinkTransform(left_arm_->getEndEffectorLink());
    Eigen::Isometry3d T_base_right = right_state->getGlobalLinkTransform(right_arm_->getEndEffectorLink());
    
    // 2. Calcular pose virtual actual (punto medio y orientación interpolada)
    Eigen::Vector3d p_virtual =( T_base_left.translation() + T_base_right.translation()) * 0.5;
    Eigen::Quaterniond q_virtual = Eigen::Quaterniond(T_base_left.rotation()).slerp(0.5, Eigen::Quaterniond(T_base_right.rotation()));
    T_base_virtual_ = Eigen::Isometry3d::Identity();
    T_base_virtual_.translation()= p_virtual;
    T_base_virtual_.linear() = q_virtual.toRotationMatrix();
}