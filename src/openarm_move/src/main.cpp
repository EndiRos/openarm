#include "openarm_move.hpp"
#include <shape_msgs/msg/solid_primitive.hpp>
#include <vector>
#include <thread>
#include <chrono>
#include <iostream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Topics_utils.hpp>
#include <tf2_ros/static_transform_broadcaster.h> 
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h> 

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
    co.id = "conveyor";
    
    RCLCPP_INFO(node->get_logger(), "[STEP 2] Cargando STL...");
    // CAMBIAR ESTA RUTA POR LA DE TU ARCHIVO STL
    // Puede ser una ruta absoluta "file:///home/user/..." o de paquete "package://paquete/meshes/..."/home/enetxeba/Documents/robotica/openarm_isaac_env/collisions
    std::string stl_path = "file:///home/enetxeba/Documents/robotica/openarm_isaac_env/collisions/conveyor_col.stl"; 
    
    shapes::Mesh* m = shapes::createMeshFromResource(stl_path);
    if (!m) {
        RCLCPP_ERROR(node->get_logger(), "No se pudo cargar el mesh desde: %s", stl_path.c_str());
        return;
    }

    // Escalar si es necesario (opcional)
    // shapes::scaleShape(m, {1.0, 1.0, 1.0});

    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
    
    RCLCPP_INFO(node->get_logger(), "[STEP 3] Configurando pose...");
    geometry_msgs::msg::Pose mesh_pose;
    mesh_pose.orientation.w = 0.70711;
    mesh_pose.orientation.z = 0.70711;
    mesh_pose.orientation.x = 0.0;
    mesh_pose.orientation.y = 0.0;

    mesh_pose.position.x = 5.01204;
    mesh_pose.position.y = -3.24295;
    mesh_pose.position.z = 0.0;

    co.meshes.push_back(mesh);
    co.mesh_poses.push_back(mesh_pose);
    co.operation = moveit_msgs::msg::CollisionObject::ADD;

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(co);

    // Usar applyCollisionObjects que es síncrono y devuelve bool
    if (planning_scene_interface.applyCollisionObjects(objects)) {
        RCLCPP_INFO(node->get_logger(), "[OK] Objeto STL agregado correctamente a la escena.");
    } else {
        RCLCPP_ERROR(node->get_logger(), "[ERROR] Falló al agregar el objeto STL.");
    }
    
    delete m; // Liberar memoria del mesh cargado
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
    OpenArmMove oa(move_group_node);
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
    
   
    

/*     if (!oa.BimanualNamedPose("pose_T","pose_T")) return 1;
    oa.BimanualExec(); 
    if (!oa.BimanualNamedPose("home","home")) return 1;
    oa.BimanualExec();*/
    oa.PtpBimanual("pose_T","pose_T");

    oa.PtpBimanual("stand_up","stand_up");
    geometry_msgs::msg::Pose pose, inter_pose, interchangeL, interchangeR, finalR;
    tf2_msgs::msg::TFMessage pos = reader->ReadTf("/tf_bootle", 5.0);
    if (pos.transforms.empty()) {
        RCLCPP_ERROR(move_group_node->get_logger(), "No se recibió TF de /tf_bootle");
        return 1;
    }
    
    geometry_msgs::msg::TransformStamped t = pos.transforms[0];
    pose.position.x = t.transform.translation.x;
    pose.position.y = t.transform.translation.y;
    pose.position.z =t.transform.translation.z;
     tf2::Quaternion q(t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,
                        t.transform.rotation.w);

    q.normalize();
    pose.orientation= tf2::toMsg(q);
    std::cout << "FIN NAMED POSE" << std::endl;
    
    //pose = createpose({0.28, 0.15, 0.44}, {0.87464, -0.23436, 0.4099, 0.10983});
    interchangeL = createpose({0.18425, 0.02312, 0.55979}, {0.59773, -0.54450, 0.39626, 0.43499});
    
    interchangeR = createpose({0.18425, 0.03668, 0.61}, {0.51542, 0.51542, 0.48409, -0.48409});     
    
    finalR = createpose({0.34436, -0.34298, 0.45341}, {0.71484, -0.14253, 0.67139, 0.13387});

    /* oa.PtpLeft(interchangeL,CLOSE);
    oa.PtpRight(interchangeR, CLOSE); */
    
    
    
    //oa.PrintPose("pose 1", pose);
    inter_pose = oa.PrevPose(pose, 0.07);
    oa.PtpLeft(inter_pose, OPEN);
    oa.PtpLeft(pose, OPEN);
 
    oa.PtpLeft(pose, CLOSE);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    pose.position.z += .05;
    oa.PtpLeft(pose,CLOSE);
    
    oa.PtpBimanual(interchangeL, oa.PrevPose(interchangeR, 0.1), CLOSE, OPEN);
    
    oa.PtpRight(interchangeR,OPEN);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    oa.PtpRight(interchangeR,CLOSE);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    geometry_msgs::msg::Pose dual_move (oa.get_current_dual_pose());
    dual_move.position.x +=.05;
    //oa.PtpBimanual(dual_move, CLOSE, CLOSE); //not viable ---> cartesian
   /*  oa.PtpLeft(interchangeL, OPEN);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    oa.PtpBimanual(oa.PrevPose(interchangeL, .05), 
                    oa.PrevPose(interchangeR, 0.1),
                    OPEN, CLOSE);
    geometry_msgs::msg::Pose final_up(finalR);
    final_up.position.z += 0.05;
    oa.PtpBimanual("stand_up", final_up, CLOSE, CLOSE);
    oa.PtpRight(finalR, CLOSE);

    oa.PtpRight(finalR, OPEN);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    finalR.position.z += .1;
    oa.PtpRight(finalR, OPEN);
    oa.PtpBimanual("stand_up","stand_up"); */

   // oa.PtpBimanual("pose_T","pose_T");
    //oa.PtpBimanual("home","home"); 
 

    // limpiar
    rclcpp::shutdown();
    exec_thread.join();
    return 0;
}
