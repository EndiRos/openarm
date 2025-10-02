#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('Simple node has been started')
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        self.get_logger().info('Timer callback')

def main():
    rclpy.init()
    node = SimpleNode()
    rclpy.spin(node)
    
    # Cleanup
    rclpy.shutdown()

if __name__ == '__main__':
    main()