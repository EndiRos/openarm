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

void OpenArmMove::PlanArm(moveit::planning_interface::MoveGroupInterface *group,                     
                        const geometry_msgs::msg::Pose& target_pos, std::string ee_link)
{
    if (!group) {
        RCLCPP_ERROR(LOGGER, "❌ Group es nulo");
        return;
    }
    
    RCLCPP_INFO(LOGGER, "🔧 planArmInternal - EE Link: %s", ee_link.c_str());
    RCLCPP_INFO(LOGGER, "📍 Target Pose: x:%.3f y:%.3f z:%.3f", 
        target_pos.position.x, target_pos.position.y, target_pos.position.z);
    
    if (!ee_link.empty()) {
      const auto &links = group->getLinkNames();
      if (std::find(links.begin(), links.end(), ee_link) != links.end()) {
        group->setEndEffectorLink(ee_link);
        RCLCPP_INFO(LOGGER, "✓ EE Link configurado: %s", group->getEndEffectorLink().c_str());
      } else {
        RCLCPP_WARN(LOGGER,
          "EE link '%s' no pertenece al grupo '%s'; usando el link por defecto '%s'",
          ee_link.c_str(), group->getName().c_str(), group->getEndEffectorLink().c_str());
      }
    }
    group->setPoseReferenceFrame(group->getPlanningFrame());
    
    geometry_msgs::msg::Pose pose = target_pos;
    tf2::Quaternion q;
    q.setX(pose.orientation.x);
    q.setY(pose.orientation.y);
    q.setZ(pose.orientation.z);
    q.setW(pose.orientation.w);

    double len = q.length();
    if (len < 1e-6) {
        // Si la orientación está "vacía", puede que contengas RPY en x,y,z -> convertir
        RCLCPP_WARN(LOGGER, "Orientación con longitud nula, interpretando como RPY (x,y,z)");
        q.setRPY(pose.orientation.x, pose.orientation.y, pose.orientation.z);
    } else if (std::fabs(len - 1.0) > 1e-6) {
        q.normalize();
    }
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    // Asegurarse de partir del estado actual y dar más intentos/tiempo para planificar
    setParam(group);

    group->setStartStateToCurrentState();

    group->setPoseTarget(pose,ee_link);
    //moveit::core::MoveItErrorCode result = group->plan(my_plan);

}

void OpenArmMove::PlanGripper(moveit::planning_interface::MoveGroupInterface* group, double gripper_state){
    
    if (gripper_state < 0.0f) gripper_state = 0.0f;
    if (gripper_state > 0.044f) gripper_state = 0.044f;
    setParam(group);
    group->setStartStateToCurrentState();
    std::vector<double> target_values;
    target_values.resize(group->getVariableCount(), gripper_state);
    group->setJointValueTarget(target_values);
}

bool OpenArmMove::PlanLeftArm(const geometry_msgs::msg::Pose& target_pos){
    PlanArm(left_arm_, target_pos, "openarm_left_hand_tcp");
    bool result = (left_arm_->plan(left_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
     if(result){
        RCLCPP_INFO(LOGGER, "✓ Left plan ok");
    }else {
        RCLCPP_ERROR(LOGGER, "✗ Left plan failed");
    }
    return result;
}

bool OpenArmMove::PlanRightArm(const geometry_msgs::msg::Pose& target_pos){
     PlanArm(right_arm_, target_pos, "openarm_right_hand_tcp");
     bool result = (right_arm_->plan(right_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
     if(result){
        RCLCPP_INFO(LOGGER, "✓ Right plan ok");
    }else {
        RCLCPP_ERROR(LOGGER, "✗ Right plan failed");
    }
    return result;
}


bool OpenArmMove::PlanLeftFull(const geometry_msgs::msg::Pose &target_pose, float gripper_state)
{

    RCLCPP_INFO(LOGGER, "=== PlanLeftFull (IK left_arm -> joint-space left_arm_full) ===");

    if (!left_arm_ || !left_arm_full_) {
        RCLCPP_ERROR(LOGGER, "MoveGroups left_arm or left_arm_full not initialized");
        return false;
    }

    // 1) Obtener estado cinemático actual desde el grupo que SÍ tiene kinematics (left_arm)
    moveit::core::RobotStatePtr kstate = left_arm_->getCurrentState();
    if (!kstate) {
        RCLCPP_ERROR(LOGGER, "No se pudo obtener RobotState del grupo 'left_arm'");
        return false;
    }

    const moveit::core::JointModelGroup *arm_jmg = kstate->getJointModelGroup("left_arm");
    if (!arm_jmg) {
        RCLCPP_ERROR(LOGGER, "No se encontró JointModelGroup 'left_arm' para IK");
        return false;
    }

    // end-effector link para left_arm (asegúrate que coincide con tu SRDF/URDF)
    std::string ee_link = left_arm_->getEndEffectorLink();

    RCLCPP_INFO(LOGGER, "Probando IK en grupo 'left_arm' (EE: %s)...", ee_link.c_str());

    // 2) Intentar IK en left_arm
    bool ik_ok = kstate->setFromIK(arm_jmg, target_pose, ee_link, 0.1);
    if (!ik_ok) {
        RCLCPP_WARN(LOGGER, "IK en 'left_arm' NO encontró solución. Intentando fallback con PlanLeftArm.");
        // fallback simple: usar planificación cartesiana sobre left_arm (ya tiene kinematics)
        bool fallback_ok = PlanLeftArm(target_pose);
        if (fallback_ok) {
            RCLCPP_INFO(LOGGER, "Fallback PlanLeftArm OK");
            return true;
        }
        RCLCPP_ERROR(LOGGER, "Fallback PlanLeftArm falló. Objetivo probablemente fuera de alcance o IK inválida.");
        return false;
    }

    RCLCPP_INFO(LOGGER, "IK en 'left_arm' encontró solución -> construyendo objetivo joint-space para 'left_arm_full'");

    // 3) Extraer posiciones del brazo (arm_jmg)
    std::vector<double> arm_positions;
    kstate->copyJointGroupPositions(arm_jmg, arm_positions);
    std::vector<std::string> arm_names = arm_jmg->getVariableNames();

    // 4) Preparar vector de posiciones para el grupo full (inicializado con estado actual)
    const moveit::core::JointModelGroup *full_jmg = kstate->getJointModelGroup("left_arm_full");
    if (!full_jmg) {
        RCLCPP_ERROR(LOGGER, "No se encontró JointModelGroup 'left_arm_full'");
        return false;
    }
    std::vector<double> full_positions(full_jmg->getVariableCount());
    kstate->copyJointGroupPositions(full_jmg, full_positions);
    std::vector<std::string> full_names = full_jmg->getVariableNames();

    // 5) Mapear valores del brazo (arm_positions) en el vector full_positions por nombre
    for (size_t i = 0; i < arm_names.size(); ++i) {
        auto it = std::find(full_names.begin(), full_names.end(), ee_link);
        if (it != full_names.end()) {
            size_t idx = std::distance(full_names.begin(), it);
            full_positions[idx] = arm_positions[i];
        } // si no está en full, lo dejamos como el valor actual
    }

    // 6) Setear valor del gripper si existe en left_arm_full
    if (gripper_state < 0.0f) gripper_state = 0.0f;
    if (gripper_state > 0.044f) gripper_state = 0.044f;
    auto git = std::find(full_names.begin(), full_names.end(), std::string("openarm_left_finger_joint1"));
    if (git != full_names.end()) {
        size_t gidx = std::distance(full_names.begin(), git);
        full_positions[gidx] = static_cast<double>(gripper_state);
    }

    
    setParam(left_arm_full_);
    // 7) Planificar en joint-space sobre left_arm_full
    left_arm_full_->setStartStateToCurrentState();
    left_arm_full_->setJointValueTarget(full_positions);

    RCLCPP_INFO(LOGGER, "Intentando planificar en joint-space sobre 'left_arm_full'...");
    bool plan_ok = (left_arm_full_->plan(left_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    if (plan_ok) {
        RCLCPP_INFO(LOGGER, "✓ Left Full joint-space plan OK (desde IK en left_arm)");
        return true;
    } else {
        RCLCPP_WARN(LOGGER, "✗ Left Full joint-space plan falló a pesar de IK. Intentando fallback a PlanLeftArm.");
        bool fallback_ok = PlanLeftArm(target_pose);
        if (fallback_ok) {
            RCLCPP_INFO(LOGGER, "Fallback PlanLeftArm OK");
            return true;
        }
        RCLCPP_ERROR(LOGGER, "Fallback PlanLeftArm también falló. Revisa alcance/colisiones/constraints.");
        return false;
    }
}

bool OpenArmMove::PlanRightFull(const geometry_msgs::msg::Pose &target_pose, float gripper_state)
{
    
    // verifica la inicializacion de los movegroups
    RCLCPP_INFO(LOGGER, "=== PlanRightFull (IK right_armfull) == -> joint-space right_arm_=");

    if (!right_arm_ || !left_arm_full_){
        RCLCPP_ERROR(LOGGER,  "MoveGroups right_arm or right_arm_full not initialized");
        return false;
    }
    moveit::core::RobotStatePtr kstate = right_arm_->getCurrentState();
    if (!kstate){
        RCLCPP_ERROR(LOGGER,"No se pudo obtener RobotState del grupo 'right_arm'");
        return false;
    }
    const moveit::core::JointModelGroup *arm_jmg = kstate->getJointModelGroup("right_arm");
    if (!arm_jmg){
        RCLCPP_ERROR(LOGGER, "No se encontró JointModelGroup 'right_arm' para IK");
        return false;
    }

    std::string ee_link = right_arm_->getEndEffectorLink();

    RCLCPP_INFO(LOGGER, "Probando IK en grupo 'right_arm' (EE: %s)...", ee_link.c_str());

    bool ik_ok = kstate->setFromIK(arm_jmg, target_pose, ee_link, 0.1);
    if (!ik_ok){
        RCLCPP_WARN(LOGGER, "IK en 'right_arm' NO encontró solución. Intentando fallback con PlanRightArm.");
        bool fallback_ok = PlanRightArm(target_pose);
        if (fallback_ok){
            RCLCPP_INFO(LOGGER, "Fallback PlanRighttArm OK" );
            return true;
        }
        RCLCPP_ERROR(LOGGER,"Fallback PlanRighttArm falló. Objetivo probablemente fuera de alcance o IK inválida.");
        return false;
    }
    RCLCPP_INFO(LOGGER,"IK en 'right_arm' encontró solución -> construyendo objetivo joint-space para 'right_arm_full'");
    std::vector<double> arm_position;
    kstate->copyJointGroupPositions(arm_jmg, arm_position);
    std::vector<std::string> arm_names = arm_jmg->getVariableNames();

    const moveit::core::JointModelGroup *full_jmg = kstate->getJointModelGroup("right_arm_full");
    if (!full_jmg) {
        RCLCPP_ERROR(LOGGER, "No se encontró JointModelGroup 'right_arm_full'");
        return false;
    }
    std::vector<double> full_position(full_jmg->getVariableCount());
    kstate->copyJointGroupPositions(full_jmg, full_position);
    std::vector<std::string> full_names = full_jmg->getVariableNames();

    for (size_t i = 0; i < arm_names.size(); i++){
        const std::string &name = arm_names[i];
        auto it = std::find(full_names.begin(),full_names.end(), name);
        if (it != full_names.end()){
            size_t idx = std::distance(full_names.begin(),it);
            full_position[idx] = arm_position[i];
        }
    }
    if (gripper_state < 0.0f) gripper_state = 0.0;
    if (gripper_state > 0.044f) gripper_state = 0.044f;
    auto git = std::find(full_names.begin(),full_names.end(), std::string("openarm_right_finger_joint1"));
    if (git != full_names.end()){
        size_t idx = std::distance(full_names.begin(),git);
        full_position[idx] = static_cast<double>(gripper_state);
    }

    
    setParam(right_arm_full_);

    right_arm_full_->setStartStateToCurrentState();
    right_arm_full_->setJointValueTarget(full_position);

    

    RCLCPP_INFO(LOGGER, "Intentando planificar en joint-space sobre 'right_arm_full'...");
    bool plan_ok = (right_arm_full_->plan(right_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
     if (plan_ok) {
        RCLCPP_INFO(LOGGER, "✓ Right Full joint-space plan OK (desde IK en right_arm)");
        return true;
    } else {
        RCLCPP_WARN(LOGGER, "✗ Right Full joint-space plan falló a pesar de IK. Intentando fallback a PlanRighttArm.");
        bool fallback_ok = PlanRightArm(target_pose);
        if (fallback_ok) {
            RCLCPP_INFO(LOGGER, "Fallback PlanRighttArm OK");
            return true;
        }
        RCLCPP_ERROR(LOGGER, "Fallback PlanRighttArm también falló. Revisa alcance/colisiones/constraints.");
        return false;
    }    
}


bool OpenArmMove::PlanBimanual(const geometry_msgs::msg::Pose &target_Left,
                               const geometry_msgs::msg::Pose &target_Right,
                               float gripper_left, float gripper_right)
{
    RCLCPP_INFO(LOGGER, "Planificar subplanes con 'left_arm' y 'right_arm'"); // 1) Planificar cada brazo por separado (para obtener la configuración objetivo por brazo)

    // Usar los grupos de brazo que tienen kinematics configuradas (left_arm / right_arm)
    bool ok_left = PlanLeftFull(target_Left, gripper_left);
    if (!ok_left){
        RCLCPP_ERROR(LOGGER, "Left arm planning failed (left_arm)");
    }

    bool ok_right = PlanRightFull(target_Right, gripper_right);
    if (!ok_right){
        RCLCPP_ERROR(LOGGER, "Left arm planning failed (left_arm)");
    }

    return ok_left && ok_right;
}

bool OpenArmMove::PlanJointTarget(moveit::planning_interface::MoveGroupInterface *group, 
                                  moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                  const std::vector<double> &joint_values)
{
    if (!group) {
        RCLCPP_ERROR(LOGGER, "Group es nulo en PlanJointTarget");
        return false;
    }

    // Verificar que el número de valores coincide con el número de joints del grupo
    const std::vector<std::string> &joint_names = group->getJointNames();
    if (joint_values.size() != joint_names.size()) {
        RCLCPP_ERROR(LOGGER, "Error: El grupo '%s' espera %zu joints, pero se recibieron %zu valores.",
            group->getName().c_str(), joint_names.size(), joint_values.size());
        return false;
    }

    RCLCPP_INFO(LOGGER, "Planificando movimiento joint-space para '%s'...", group->getName().c_str());

    // Configurar parámetros de planificación
    setParam(group);
    
    group->setStartStateToCurrentState();
    
    // Establecer el objetivo (vector de doubles)
    group->setJointValueTarget(joint_values);

    // Planificar
    moveit::core::MoveItErrorCode result = group->plan(plan);
    
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(LOGGER, "✓ Plan joint-space exitoso");
        return true;
    } else {
        RCLCPP_ERROR(LOGGER, "✗ Falló la planificación joint-space (código: %d)", result.val);
        return false;
    }
}