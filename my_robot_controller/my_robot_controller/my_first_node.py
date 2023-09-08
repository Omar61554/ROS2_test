#!/usr/bin/env python3

import rclpy                                      # Import the rclpy module
from rclpy.node import Node                       # Import the Node class from rclpy

class MyNode(Node):                               # Create a custom class MyNode thatS inherits from Node
    
    def __init__(self):
        super().__init__("first_node")            # Initialize the node with the name "first_node"
        self.create_timer(1.0,self.timer_callback) # Log a message "hello from ros2" at the info level

    def timer_callback(self):
        
        self.get_logger().info("HI!")
def main(args=None):
    rclpy.init(args=args)                         # Initialize the ROS2 communication system
    node = MyNode()                               # Create an instance of the MyNode class
    
    rclpy.spin(node)                              # Keep the script running until the node is shutdown
    
    node.destroy_node()                           # Clean up the node
    rclpy.shutdown()                              # Shutdown the ROS2 communication system

                   # block ensures that the main function is only executed if the script is run directly (not imported as a module).
if __name__ == '__main__':
    main()                                        # Call the main function to start the ROS2 node