#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class PlannedPath(Node):
    def __init__(self):
        super().__init__("planned_path")
        self.get_logger().info("Started planned path...")

        self._pulisher = self.create_publisher(AckermannDriveStamped, "drive", 1)
    
    def drive(self):
        msg = AckermannDriveStamped()
        
        msg.header.stamp.sec = 10.0
        msg.header.stamp.nanosec = 0.0
        msg.header.stamp.frame_id = "planned_path"
        msg.drive.steering_angle = 0
        msg.drive.steering_angle_velocity = 0
        msg.drive.speed = 1
        msg.drive.acceleration = 0
        msg.drive.jerk = 0

        self._publisher.publish(msg)
        

def main():
    rclpy.init(args=args)
    node = PlannedPath()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
