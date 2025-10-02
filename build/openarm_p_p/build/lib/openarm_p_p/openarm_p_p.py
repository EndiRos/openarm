#!/usr/bin/env python3
import rclpy
import sys
import subprocess
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient

class MoveItPythonNode(Node):
    def __init__(self):
        super().__init__("hello_moveit_python")
        self.get_logger().info("Initializing MoveIt Python Node")
        
        # Crear cliente de acción para MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, "move_action")
        
        # Esperar a que el servidor esté disponible
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server not available!")
            return
            
        self.get_logger().info("MoveGroup action server connected!")

def main():
    # Inicializar ROS
    rclpy.init()
    
    try:
        # Crear y usar el nodo
        node = MoveItPythonNode()
        
        # Verificar si la interfaz de MoveIt está disponible
        node.get_logger().info("Testing MoveIt with command-line tools")
        
        # Usar línea de comandos para mover el robot
        try:
            # Esto es solo un ejemplo - deberías ajustarlo según tu configuración
            cmd = ["ros2", "run", "moveit_ros_move_group", "list_move_group", "--ros-args", "-p", "group:=left_arm"]
            result = subprocess.run(cmd, capture_output=True, text=True)
            node.get_logger().info(f"Command output: {result.stdout}")
            if result.returncode != 0:
                node.get_logger().error(f"Command failed: {result.stderr}")
        except Exception as e:
            node.get_logger().error(f"Failed to run command: {e}")
            
        # Esperar a que el nodo sea interrumpido
        rclpy.spin(node)
    
    except Exception as e:
        print(f"Error: {e}")


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

