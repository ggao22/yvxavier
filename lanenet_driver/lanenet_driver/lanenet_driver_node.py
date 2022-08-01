#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from points_vector.msg import PointsVector
from ackermann_msgs.msg import AckermannDriveStamped

from .stanley_controller import cubic_spline_planner
from .stanley_controller.stanley_controller import State, calc_target_index, pid_control, stanley_control


class LanenetDriver(Node):

    def __init__(self):
        super().__init__("lanenet_driver")
        self.get_logger().info("Started lanenet driver...")

        self.subscriber_ = self.create_subscription(PointsVector, '/lanenet_path', self.driver_callback, 1)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, "drive", 1)

        self.drive_exists = False

        self.ctl_loop = self.create_timer(0.05,self.main_control)
        
    def driver_callback(self, data):
        self.drive_exists = True
        self.get_logger().info("New data received...")

        vector = data.points
        x_coeff = data.x_coeff
        self.ax = []
        self.ay = []

        for pt in vector:
            self.ax.append(pt.x)
            self.ay.append(pt.y)
        
        self.ax.reverse()
        self.ay.reverse()
        
        self.cx, self.cy, self.cyaw, ck, s = cubic_spline_planner.calc_spline_course(
            self.ax, self.ay, ds=0.001)

        self.target_speed = 0.8 / 3.6  # [m/s]

        # Initial state
        self.state = State(x=640*x_coeff, y=0, yaw=np.radians(90.0), v=0.8)

        self.last_idx = len(self.cx) - 1
        self.target_idx, _ = calc_target_index(self.state, self.cx, self.cy)

    def main_control(self):
        if self.drive_exists and self.last_idx > self.target_idx:
            self.get_logger().info("Stanley controlling...")
            self.ai = pid_control(self.target_speed, self.state.v)
            self.di, self.target_idx = stanley_control(self.state, self.cx, self.cy, self.cyaw, self.target_idx)
            delta, v = self.state.update(self.ai, self.di)
            self.drive_with_steer(delta, v)

    def drive_with_steer(self, steering_angle, velocity):
        msg = AckermannDriveStamped()

        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = "lanenet_drive"
        msg.drive.steering_angle = steering_angle
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.speed = velocity
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0

        self.get_logger().info("Publishing on drive...")
        self.publisher_.publish(msg)
        

def main(args=None):
    try:
        rclpy.init(args=args)

        lanenet_driver = LanenetDriver()

        rclpy.spin(lanenet_driver)
        lanenet_driver.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

