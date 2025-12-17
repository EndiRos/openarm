#include "openarm_move.hpp"
#include <shape_msgs/msg/solid_primitive.hpp>
#include <vector>
#include <thread>
#include <chrono>
#include <iostream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Topics_utils.hpp>
#include <tf2_ros/static_transform_broadcaster.h> 

geometry_msgs::msg::Pose createpose(const std::vector<double>& pos, const std::vector<double>& rot)
{
    geometry_msgs::msg::Pose ret;
    ret.position.x = pos[0];
    ret.position.y =pos[1];
    ret.position.z = pos[2];

    ret.orientation.x = rot[0];
    ret.orientation.y = rot[1];
    ret.orientation.z = rot[2];
    ret.orientation.w = rot[3];

    tf2::Quaternion q(ret.orientation.x,
                    ret.orientation.y,
                    ret.orientation.z,
                    ret.orientation.w);
    if (std::fabs(q.length()) < 1e-6) {
    std::cout <<  "Orientación casi nula; interpretando x,y,z como RPY"<< std::endl;
      q.setRPY(ret.orientation.x, ret.orientation.y, ret.orientation.z);
    }
    q.normalize();
    ret.orientation = tf2::toMsg(q);
    return ret;
}


void create_obstacle(rclcpp::Node::SharedPtr node,
                moveit::planning_interface::PlanningSceneInterface &planning_scene_interface){
    
    // Aumentar espera inicial para asegurar conexión
    RCLCPP_INFO(node->get_logger(), "[INIT] Esperando a PlanningSceneInterface...");
    std::this_thread::sleep_for(std::chrono::seconds(2)); 

    RCLCPP_INFO(node->get_logger(), "[STEP 1] Creando CollisionObject...");
    moveit_msgs::msg::CollisionObject co;
    co.header.frame_id = "world"; 
    co.header.stamp = node->now(); // Importante poner timestamp actual
    co.id = "obst_box_1";
    
    RCLCPP_INFO(node->get_logger(), "[STEP 2] Definiendo geometría...");
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions.resize(3);
    box.dimensions[0] = 0.95; // x
    box.dimensions[1] = 1.5;  // y
    box.dimensions[2] = 0.02; // z (altura)
    RCLCPP_INFO(node->get_logger(), "[STEP 3] Configurando pose...");
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 1.08735;
    box_pose.position.y = 0.0;
    // centrar la caja sobre el suelo: z = height/2
    box_pose.position.z = 0.75171;

    co.primitives.push_back(box);
    co.primitive_poses.push_back(box_pose);
    co.operation = moveit_msgs::msg::CollisionObject::ADD;

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(co);

    // Usar applyCollisionObjects que es síncrono y devuelve bool
    if (planning_scene_interface.applyCollisionObjects(objects)) {
        RCLCPP_INFO(node->get_logger(), "[OK] Objeto agregado correctamente a la escena.");
    } else {
        RCLCPP_ERROR(node->get_logger(), "[ERROR] Falló al agregar el objeto.");
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // arrancar executor ANTES de crear el obstáculo para garantizar conexión publishers/subscribers
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread exec_thread([&executor]() { executor.spin(); });
    auto reader = std::make_shared<ReadOnce>(move_group_node);
    tf2_msgs::msg::TFMessage base_pos = reader->ReadTf("/isaac/tf_robot", 5.0);
    
    if (!base_pos.transforms.empty()){
         geometry_msgs::msg::TransformStamped t = base_pos.transforms[0];

        // Aseguramos que los frames sean correctos para MoveIt (world -> base_link)
        t.header.frame_id = "world";
        t.child_frame_id = "base_link";
        t.header.stamp = move_group_node->now();

        // Publicar la transformación estática
        static auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(move_group_node);
        tf_broadcaster->sendTransform(t);

        RCLCPP_INFO(move_group_node->get_logger(), 
            "Base movida a: [%.2f, %.2f, %.2f]", 
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
        
        // Esperar un poco para que el árbol de TF se actualice
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } else {
        RCLCPP_WARN(move_group_node->get_logger(), "No se recibió TF o la lista estaba vacía.");
    }
    
    // ahora crear obstáculo (con executor ya en marcha)
    create_obstacle(move_group_node, planning_scene_interface);
    
    OpenArmMove oa(move_group_node);
    

/*     if (!oa.BimanualNamedPose("pose_T","pose_T")) return 1;
    oa.BimanualExec(); 
    if (!oa.BimanualNamedPose("home","home")) return 1;
    oa.BimanualExec();*/
    oa.PtpBimanual("pose_T","pose_T");

    oa.PtpBimanual("stand_up","stand_up");
    geometry_msgs::msg::Pose pose, inter_pose, interchangeL, interchangeR, finalR;
    pose = reader->ReadPose("/tcp",5.0);

    std::cout << "FIN NAMED POSE" << std::endl;
    
    interchangeL = createpose({0.1638, 0.0, 0.4977},
                             {0.64224, -0.64224, 0.29585, 0.29585});
    interchangeR = createpose({0.15708, 0.019, 0.47148},
                             {-0.5, -0.5, -0.5, 0.5});        
    finalR = createpose({0.35, -0.21, 0.4},
                             {0.70711, -0.0, 0.70711, 0.0}); 
   
    
    inter_pose = oa.PrevPose(pose, 0.1);

    oa.PtpLeft(inter_pose, OPEN);
    oa.PtpLeft(pose, OPEN);
 
    oa.PtpLeft(pose, CLOSE);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    oa.PtpBimanual(interchangeL,oa.PrevPose(interchangeR, 0.1), CLOSE, OPEN );
    
    oa.PtpRight(interchangeR,OPEN);
    
    oa.PtpRight(interchangeR,CLOSE);
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    oa.PtpLeft(interchangeL, OPEN);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    oa.PtpBimanual(oa.PrevPose(interchangeL, .05), 
                    oa.PrevPose(interchangeR, 0.1),
                    OPEN, CLOSE);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    oa.PtpBimanual("stand_up", finalR, CLOSE, CLOSE);

    std::this_thread::sleep_for(std::chrono::seconds(2));
    oa.PtpRight(finalR, OPEN);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    oa.PtpRight(oa.PrevPose(finalR, .1), OPEN);
    oa.PtpBimanual("stand_up","stand_up");

    oa.PtpBimanual("pose_T","pose_T");
    oa.PtpBimanual("home","home");
 

    // limpiar
    rclcpp::shutdown();
    exec_thread.join();
    return 0;
}
