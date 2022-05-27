#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class PlannedPath(Node):
    def __init__(self):
        super().__init__("planned_path")
        self.get_logger().info("Started planned path...")
        

def main():
    rclpy.init(args=args);
    node = PlannedPath()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
