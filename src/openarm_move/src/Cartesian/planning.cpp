
#include "cartesian.hpp"


bool Cartesian::PlanArm(moveit::planning_interface::MoveGroupInterface *group,\
                moveit_msgs::msg::RobotTrajectory &trayectory,\
                moveit::planning_interface::MoveGroupInterface::Plan &plan,\
                 const std::vector<geometry_msgs::msg::Pose> &points)
{
    double fraction = group->computeCartesianPath(
                    points,
                    eef_step_,
                    jump_thershold_,
                    trayectory);
    if (fraction > 0.9 ){
        plan.trajectory_ = trayectory;
    }else
    {
        RCLCPP_WARN(LOGGER,"Cant calculate cartesian trayectory completely");
        return false;
    }
    return true;
};

bool Cartesian::PlanLeftArm(const std::vector<geometry_msgs::msg::Pose> &waypoints)
{
    return PlanArm(left_arm_, left_trayectoty_, left_plan_, waypoints);

}
bool Cartesian::PlanRightArm(const std::vector<geometry_msgs::msg::Pose>& waypoints) {
    PlanArm(right_arm_, right_trayectoty_, right_plan_, waypoints);
}

bool Cartesian::PlanLeftArm_full(const std::vector<geometry_msgs::msg::Pose>& waypoints, double left_gripper) {
         PlanArm(left_arm_full_, left_trayectoty_, left_plan_, waypoints);
         
    }

bool Cartesian::PlanRightArm_full(
        const std::vector<geometry_msgs::msg::Pose>& waypoints,
        double gripper_val) {
      return false;
    }

bool Cartesian::PlanBimanual(
        const std::vector<geometry_msgs::msg::Pose>& left_waypoints,
        const std::vector<geometry_msgs::msg::Pose>& right_waypoints,
        double left_grippe, double right_gripper) {
      return false;
    }
