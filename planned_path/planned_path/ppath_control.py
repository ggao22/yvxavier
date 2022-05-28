#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class PlannedPath(Node):

    def __init__(self):
        super().__init__("planned_path")
        self.get_logger().info("Started planned path...")

        self._publisher = self.create_publisher(AckermannDriveStamped, "drive", 1)
        self.period_output = self.create_timer(0.5,self.drive)
    
    def drive(self):
        msg = AckermannDriveStamped()

        msg.header.stamp.sec = 10
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = "planned_path"
        msg.drive.steering_angle = 0.0
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.speed = 1.0
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0

        self._publisher.publish(msg)
        self.get_logger().info('Publishing: planned path...')
        

def main(args=None):
    try:
        rclpy.init(args=args)
        path_publisher = PlannedPath()
        rclpy.spin(path_publisher)
        path_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

