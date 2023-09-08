#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute


class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("draw_circle")

        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.teleport_abs_ = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")

        while not self.teleport_abs_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for teleport_absolute service...")

        self.reset_turtle()

        self.timer_ = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("draw circle node has been started")

    def reset_turtle(self):
        request = TeleportAbsolute.Request()
        request.x = 5.4
        request.y = 5.4
        request.theta = 0.0

        future = self.teleport_abs_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Turtle position reset successfully")
        else:
            self.get_logger().error("Failed to reset turtle position")

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 5.0
        msg.angular.z = 1.0
        self.cmd_vel_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()