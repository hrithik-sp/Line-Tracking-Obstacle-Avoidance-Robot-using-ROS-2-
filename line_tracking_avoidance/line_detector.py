#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool, String
from cv_bridge import CvBridge

class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')
        self.bridge = CvBridge()
        self.declare_parameter('roi_top_ratio', 0.55)
        self.declare_parameter('min_area', 180)
        self.roi_top_ratio = float(self.get_parameter('roi_top_ratio').value)
        self.min_area = int(self.get_parameter('min_area').value)

        # White line detection in HSV:
        # Ground plane in SDF: RGB(0.3, 0.3, 0.3) → HSV V ≈ 76  (dark grey)
        # White line in SDF:   RGB(1.0, 1.0, 1.0) → HSV V ≈ 255 (pure white)
        # Setting V_min=200 cleanly separates line from ground.
        self.lower_white = np.array([0,   0, 200])
        self.upper_white = np.array([180, 40, 255])

        self.pub_error    = self.create_publisher(Float32, '/line_error',     10)
        self.pub_detected = self.create_publisher(Bool,    '/line_detected',  10)
        self.pub_side     = self.create_publisher(String,  '/line_side',      10)
        self.pub_debug    = self.create_publisher(Image,   '/line_debug_img', 10)

        # Topic: /camera/rgb/image_raw is what the bridge remaps to
        self.create_subscription(
            Image, '/camera/rgb/image_raw', self.cb, 10)
        self.get_logger().info('Line Detector started — waiting for camera on /camera/rgb/image_raw')

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w  = frame.shape[:2]

        # Use only the bottom band where line should appear.
        y0 = int(h * self.roi_top_ratio)
        roi = frame[y0:h, :]
        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_white, self.upper_white)

        # Morphology: remove noise, fill gaps
        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Only accept contours larger than min_area pixels.
        valid = [c for c in contours if cv2.contourArea(c) > self.min_area]

        if valid:
            largest = max(valid, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] > 0:
                cx_roi = int(M['m10'] / M['m00'])
                cx = cx_roi
                error = float(cx - w // 2) / float(w // 2)   # normalised to [-1, 1]

                if cx < w // 3:
                    side = 'left'
                elif cx > 2 * w // 3:
                    side = 'right'
                else:
                    side = 'center'

                self.pub_error.publish(Float32(data=error))
                self.pub_detected.publish(Bool(data=True))
                self.pub_side.publish(String(data=side))
                debug = frame.copy()
                cv2.line(debug, (w // 2, y0), (w // 2, h - 1), (255, 0, 0), 2)
                cv2.circle(debug, (cx, y0 + int(roi.shape[0] * 0.5)), 6, (0, 255, 0), -1)
                self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8'))
                self.get_logger().info(
                    f'Line: cx={cx} err={error:.0f} side={side} '
                    f'area={cv2.contourArea(largest):.0f}',
                    throttle_duration_sec=1)
                return

        # No line found
        self.pub_error.publish(Float32(data=0.0))
        self.pub_detected.publish(Bool(data=False))
        self.pub_side.publish(String(data='none'))
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        self.get_logger().warn('No line detected', throttle_duration_sec=2)

def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()