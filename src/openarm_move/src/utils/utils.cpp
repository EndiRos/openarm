

#include "utils.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


Utils::Utils(MotionStrategy* strategy) : strategy_(strategy) {}

Utils::~Utils(){}

void Utils::PrintRobotInfo() {
    RCLCPP_INFO(LOGGER, "=== LEFT ARM ===");
    RCLCPP_INFO(LOGGER, "Planning frame: %s", strategy_->left_arm_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", strategy_->left_arm_->getEndEffectorLink().c_str());
    
    geometry_msgs::msg::PoseStamped left_pose = strategy_->left_arm_->getCurrentPose();
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
    RCLCPP_INFO(LOGGER, "Planning frame: %s", strategy_->right_arm_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", strategy_->right_arm_->getEndEffectorLink().c_str());
    
    geometry_msgs::msg::PoseStamped right_pose = strategy_->right_arm_->getCurrentPose();
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

std::vector<geometry_msgs::msg::Pose> Utils::GetCurrentPoseLeft() {
    std::vector<geometry_msgs::msg::Pose> result;
    
    if (!strategy_->left_arm_full_) {
        RCLCPP_ERROR(LOGGER, "left_arm_full_ no inicializado en GetCurrentPoseLeft()");
        return result;
    }
    
    try {
        geometry_msgs::msg::PoseStamped ps = strategy_->left_arm_full_->getCurrentPose();
        result.push_back(ps.pose);
        return result;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "Excepción al obtener pose actual del brazo izquierdo: %s", e.what());
        return result;
    }
}

std::vector<geometry_msgs::msg::Pose> Utils::GetCurrentPoseRight() {
    std::vector<geometry_msgs::msg::Pose> result;
    
    if (!strategy_->right_arm_full_) {
        RCLCPP_ERROR(LOGGER, "right_arm_full_ no inicializado en GetCurrentPoseRight()");
        return result;
    }
    
    try {
        geometry_msgs::msg::PoseStamped ps = strategy_->right_arm_full_->getCurrentPose();
        result.push_back(ps.pose);
        return result;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "Excepción al obtener pose actual del brazo derecho: %s", e.what());
        return result;
    }
}

std::vector<geometry_msgs::msg::Pose> Utils::AproachPoint(const std::vector<geometry_msgs::msg::Pose> target_pos, float dist) {

    
    std::vector<geometry_msgs::msg::Pose> ret(target_pos);
    tf2::Quaternion q;
    q.setX(ret.back().orientation.x);
    q.setY(ret.back().orientation.y);
    q.setZ(ret.back().orientation.z);
    q.setW(ret.back().orientation.w);

    tf2::Matrix3x3 m(q);
    tf2::Vector3 z_axis = m.getColumn(2);
    ret.back().position.x = ret.back().position.x - z_axis.x() * dist;
    ret.back().position.y = ret.back().position.y - z_axis.y() * dist;
    ret.back().position.z = ret.back().position.z - z_axis.z() * dist;
    return ret; 
}

void Utils::PrintPose(const std::string& pose_name, const geometry_msgs::msg::Pose& pose) {
    RCLCPP_INFO(LOGGER, "Pose name: %s", pose_name.c_str());
    RCLCPP_INFO(LOGGER, "Position: x:%f y:%f z:%f",
        pose.position.x, pose.position.y, pose.position.z);
    RCLCPP_INFO(LOGGER, "Orientation (Quaternion) x:%f, y:%f, z:%f, w:%f",
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
}

std::vector<geometry_msgs::msg::Pose> Utils::TF2Vector(const tf2_msgs::msg::TFMessage& tf) {
    std::vector<geometry_msgs::msg::Pose> ret;
    ret.reserve(tf.transforms.size());

    for (const auto& ts : tf.transforms) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = ts.transform.translation.x;
        pose.position.y = ts.transform.translation.y;
        pose.position.z = ts.transform.translation.z;

        tf2::Quaternion q(ts.transform.rotation.x,
                          ts.transform.rotation.y,
                          ts.transform.rotation.z,
                          ts.transform.rotation.w);
        q.normalize();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        ret.push_back(pose);
    }

    return ret;
}

std::vector<geometry_msgs::msg::Pose> createpose(const std::vector<double>& pos, const std::vector<double>& rot)
{
    std::vector<geometry_msgs::msg::Pose> ret(1);
    geometry_msgs::msg::Pose& pose = ret.front();

    pose.position.x = pos.size() > 0 ? pos[0] : 0.0;
    pose.position.y = pos.size() > 1 ? pos[1] : 0.0;
    pose.position.z = pos.size() > 2 ? pos[2] : 0.0;

    tf2::Quaternion q;
    if (rot.size() >= 4 && std::fabs(tf2::Quaternion(rot[0], rot[1], rot[2], rot[3]).length()) >= 1e-6) {
        q.setX(rot[0]);
        q.setY(rot[1]);
        q.setZ(rot[2]);
        q.setW(rot[3]);
    } else if (rot.size() >= 3) {
        std::cout << "Orientación casi nula; interpretando x,y,z como RPY" << std::endl;
        q.setRPY(rot[0], rot[1], rot[2]);
    } else {
        q.setRPY(0.0, 0.0, 0.0);  // identidad
    }

    q.normalize();
    pose.orientation = tf2::toMsg(q);
    return ret;
}