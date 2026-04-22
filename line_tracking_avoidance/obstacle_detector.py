#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('front_fov_deg', 60.0)
        self.declare_parameter('side_fov_deg', 70.0)
        self.safe_dist = self.get_parameter(
            'safe_distance').get_parameter_value().double_value
        self.front_fov = math.radians(
            self.get_parameter('front_fov_deg').get_parameter_value().double_value)
        self.side_fov = math.radians(
            self.get_parameter('side_fov_deg').get_parameter_value().double_value)

        self.pub_detected  = self.create_publisher(Bool,    '/obstacle_detected',  10)
        self.pub_distance  = self.create_publisher(Float32, '/obstacle_distance',  10)
        self.pub_direction = self.create_publisher(String,  '/obstacle_direction', 10)
        self.pub_front     = self.create_publisher(Float32, '/front_distance',     10)
        self.pub_left      = self.create_publisher(Float32, '/left_distance',      10)
        self.pub_right     = self.create_publisher(Float32, '/right_distance',     10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info(f'Obstacle Detector started. Safe dist: {self.safe_dist}m')

    def scan_callback(self, msg):
        r = np.array(msg.ranges)
        r = np.where(np.isinf(r) | np.isnan(r), 10.0, r)
        # Compute sector minima from scan angles instead of fixed indices.
        # This works even when scan starts at -pi and wraps to +pi.
        angles = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        wrapped = np.arctan2(np.sin(angles), np.cos(angles))

        half_front = self.front_fov * 0.5
        front_mask = np.abs(wrapped) <= half_front
        left_mask = (wrapped > half_front) & (wrapped <= (half_front + self.side_fov))
        right_mask = (wrapped < -half_front) & (wrapped >= -(half_front + self.side_fov))

        front = float(np.min(r[front_mask])) if np.any(front_mask) else 10.0
        left = float(np.min(r[left_mask])) if np.any(left_mask) else 10.0
        right = float(np.min(r[right_mask])) if np.any(right_mask) else 10.0

        blocked = front < self.safe_dist
        self.pub_detected.publish(Bool(data=bool(blocked)))
        self.pub_distance.publish(Float32(data=front))
        self.pub_front.publish(Float32(data=front))
        self.pub_left.publish(Float32(data=left))
        self.pub_right.publish(Float32(data=right))

        d = String()
        if blocked:
            d.data = 'left' if left > right else 'right'
            self.get_logger().info(
                f'Obstacle! front={front:.2f}m left={left:.2f}m '
                f'right={right:.2f}m → avoid {d.data}',
                throttle_duration_sec=1)
        else:
            d.data = 'none'
        self.pub_direction.publish(d)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ObstacleDetector())
    rclpy.shutdown()

if __name__ == '__main__':
    main()