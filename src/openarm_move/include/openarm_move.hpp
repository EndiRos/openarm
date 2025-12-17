
#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <tf2_ros/transform_listener.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <logger.hpp>


#define OPEN 0.044f
#define CLOSE 0.0f

class OpenArmMove
{
    private:
        
        double  MaxVelocityScalingFactor_;
        double  MaxAccelerationScalingFactor_;
        int     NumPlanningAttempts_;
        double  PlanningTime_;

        rclcpp::Node::SharedPtr node_;
     /*    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr tf_subscription_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_; */
       
        moveit::planning_interface::MoveGroupInterface* left_arm_;
        moveit::planning_interface::MoveGroupInterface* right_arm_;
        
        moveit::planning_interface::MoveGroupInterface* left_gripper_;
        moveit::planning_interface::MoveGroupInterface* right_gripper_;

        moveit::planning_interface::MoveGroupInterface* left_arm_full_;
        moveit::planning_interface::MoveGroupInterface* right_arm_full_;
        moveit::planning_interface::MoveGroupInterface* bimanual_;
        
        const moveit::core::JointModelGroup* left_joint_model_group_;
        const moveit::core::JointModelGroup* right_joint_model_group_;
        const moveit::core::JointModelGroup* bimanual_joint_model_group;
       

        geometry_msgs::msg::Pose target_pose_left;
        geometry_msgs::msg::Pose target_pose_right;


        moveit::planning_interface::MoveGroupInterface::Plan left_plan_;
        moveit::planning_interface::MoveGroupInterface::Plan right_plan_;
        moveit::planning_interface::MoveGroupInterface::Plan bimanual_plan_;

         void setParam(moveit::planning_interface::MoveGroupInterface* group);

    public:
        OpenArmMove(rclcpp::Node::SharedPtr node);
        ~OpenArmMove();

        void PtpBimanual(const geometry_msgs::msg::Pose& target_left,
                        const geometry_msgs::msg::Pose& target_right,
                        float gripper_l, float gripper_r);

        void PtpBimanual(const geometry_msgs::msg::Pose& target_left,
                        std::string pose_right, float gripper_l, float gripper_r);

        void PtpBimanual(std::string pose_left,
                        const geometry_msgs::msg::Pose& target_right,
                        float gripper_l, float gripper_r);

        void PtpBimanual(std::string pose_left, std::string pose_right);

        void PtpLeft(const geometry_msgs::msg::Pose &target, float gripper);

        void PtpLeft(std::string pose);

        void PtpRight(const geometry_msgs::msg::Pose& target, float gripper);

        void PtpRight(std::string pose);

        void PlanArm(moveit::planning_interface::MoveGroupInterface *group,\
                const geometry_msgs::msg::Pose &target_pos, \
                std::string ee_link);
        void PrintRobotInfo();
        bool PlanLeftArm(const geometry_msgs::msg::Pose& target_pose);
        bool PlanRightArm(const geometry_msgs::msg::Pose& target_pose);
        bool PlanLeftFull(const geometry_msgs::msg::Pose& target_pose, float gripper);
        bool PlanRightFull(const geometry_msgs::msg::Pose& target_pose, float gripper);
        bool PlanBimanual(const geometry_msgs::msg::Pose & target_Left, const geometry_msgs::msg::Pose & target_Right,\
                         float gripper_left, float gripper_right );
        void OpenLeftGripper(); 
        void CloseLeftGripper();
        void OpenRightGripper();
        
        void CloseRightGripper();
        bool PlanToNamedPose(moveit::planning_interface::MoveGroupInterface *group,\
            moveit::planning_interface::MoveGroupInterface::Plan& plan_,\
            const std::string pose_name);

        bool LeftToNamedPose(const std::string &pose_name);
        bool RightToNamedPose(const std::string &pose_name);
        bool LeftFullToNamedPose(const std::string &pose_name);
        bool PlanJointTarget(moveit::planning_interface::MoveGroupInterface *group, moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::vector<double> &joint_values);
        bool RightFullToNamedPose(const std::string &pose_name);
        bool BimanualNamedPose(std::string left_pose, std::string right_pose);
        geometry_msgs::msg::Pose PrevPose(const geometry_msgs::msg::Pose &Target_pos, float dist);

       

        geometry_msgs::msg::Pose get_target_pose_left();
        geometry_msgs::msg::Pose get_target_pose_right();
        geometry_msgs::msg::Pose get_current_pose_right();
        geometry_msgs::msg::Pose get_current_pose_left();

        void LeftArmExec();
        void RightArmExec();
        bool BimanualExec();
        
        
        
        static const tf2::Quaternion q_offset;
      
        // devuelve el planning frame del brazo izquierdo (usarlo para colocar objetos en la escena)
        std::string getPlanningFrameLeft() const;
        bool mergePlansWithInterpolation(const moveit::planning_interface::MoveGroupInterface::Plan &left_plan, const moveit::planning_interface::MoveGroupInterface::Plan &right_plan, moveit::planning_interface::MoveGroupInterface::Plan &out_plan, double right_delay_seconds, double min_dt);
};