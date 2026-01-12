#pragma once
#include <vector> // Necesario para std::vector
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp> // Necesario para Pose
#include <rclcpp/rclcpp.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <geometry_msgs/msg/transform.hpp>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <logger.hpp>

#define OPEN 0.044f
#define CLOSE 0.0f

/* static double sample_joint_from_traj(const trajectory_msgs::msg::JointTrajectory &traj,
                                     const std::string &joint_name, double t);
static double duration_to_seconds(const builtin_interfaces::msg::Duration &d);
static builtin_interfaces::msg::Duration seconds_to_duration(double t); */

class Utils;

class MotionStrategy {
    public:
        friend class Utils;
        
        MotionStrategy(rclcpp::Node::SharedPtr node);
        // El destructor virtual es CRUCIAL para polimorfismo
        virtual ~MotionStrategy(); 

        // Definimos la interfaz común usando vector (el caso más general)
        // = 0 lo hace "puro virtual" (abstracto), obligando a Ptp y Cartesian a implementarlo.
        virtual bool PlanLeftArm(const std::vector<geometry_msgs::msg::Pose>& waypoints) = 0;
        virtual bool PlanRightArm(const std::vector<geometry_msgs::msg::Pose>& waypoints) = 0;
        
        // Para las versiones full, pasamos el gripper aparte
        virtual bool PlanLeftArm_full(const std::vector<geometry_msgs::msg::Pose>& waypoints, double gripper_val) = 0;
        virtual bool PlanRightArm_full(const std::vector<geometry_msgs::msg::Pose>& waypoints, double gripper_val) = 0;
        
        // Bimanual podría recibir dos vectores
        virtual bool PlanBimanual(const std::vector<geometry_msgs::msg::Pose>& left_waypoints, 
                          const std::vector<geometry_msgs::msg::Pose>& right_waypoints, double left_gripper, double right_gripper) = 0;
    
        bool ExecuteLeft();
        bool ExecuteRight();
        bool ExecuteBimanual();

        bool mergePlansWithInterpolation(double right_delay_seconds, double min_dt);
        
        void setParam(moveit::planning_interface::MoveGroupInterface* group);
        // Métodos de información y utilidad común a ambas estrategias
        // para capturar de un topic de isaac u otro lado, hay que hacerlo aparte 
        /* std::vector<geometry_msgs::msg::Pose> GetTargetPoseLeft();
        std::vector<geometry_msgs::msg::Pose> GetTargetPoseRight(); */
        
        
        
        //std::vector<geometry_msgs::msg::Pose> GetCurrentDualPose(); calculo dual

       protected:
        rclcpp::Node::SharedPtr node_;
        
        // Punteros a los grupos
        moveit::planning_interface::MoveGroupInterface* left_arm_;
        moveit::planning_interface::MoveGroupInterface* right_arm_;
        moveit::planning_interface::MoveGroupInterface* left_gripper_;
        moveit::planning_interface::MoveGroupInterface* right_gripper_;
        moveit::planning_interface::MoveGroupInterface* left_arm_full_;
        moveit::planning_interface::MoveGroupInterface* right_arm_full_;
        moveit::planning_interface::MoveGroupInterface* bimanual_;

        // Planes almacenados para la ejecución posterior
        moveit::planning_interface::MoveGroupInterface::Plan left_plan_;
        moveit::planning_interface::MoveGroupInterface::Plan right_plan_;
        moveit::planning_interface::MoveGroupInterface::Plan bimanual_plan_;
        moveit::planning_interface::MoveGroupInterface::Plan left_gripper_plan_;
        moveit::planning_interface::MoveGroupInterface::Plan right_gripper_plan_;

        double  MaxVelocityScalingFactor_;
        double  MaxAccelerationScalingFactor_;
        unsigned int NumPlanningAttempts_;
        double  PlanningTime_;

};