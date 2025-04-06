#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np
import time

class DiameterPublisher(Node):
    def __init__(self):
        super().__init__('diameter_publisher')
        self.publisher_ = self.create_publisher(Float32, 'diameter', 10)
        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback)  # 15 FPS

        self.calibration_data = np.load('/home/mo/ws_python/src/orange_detector/orange_detector/camera_calibration.npz')
        self.camera_matrix = self.calibration_data['camera_matrix']
        self.dist_coeffs = self.calibration_data['dist_coeffs']

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.cap = cv2.VideoCapture(2)

        self.ripe_orange_lower = (10, 150, 150)
        self.ripe_orange_upper = (25, 255, 255)
        self.MARKER_SIZE_CM = 5.25

        self.prev_diameter = 0.0
        self.last_frame_time = time.time()

    def find_diameter_circle(self, circle, ratio):
        if circle is not None:
            x, y, radius = circle
            return 2 * radius / ratio
        return 0

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Couldn't capture frame")
            return

        frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        frame = cv2.resize(frame, (640, 480))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.ripe_orange_lower, self.ripe_orange_upper)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        orange_position = None
        radius = 0

        if contours:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > 10:
                orange_position = (x, y)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        diameter_cm = 0.0
        if ids is not None:
            marker_corners = corners[0][0]
            marker_width = np.linalg.norm(marker_corners[0] - marker_corners[1])
            ratio = marker_width / self.MARKER_SIZE_CM
            if orange_position:
                diameter_cm = self.find_diameter_circle((x, y, radius), ratio)
                if abs(diameter_cm - self.prev_diameter) > 0.2:
                    self.get_logger().info(f"Diameter: {diameter_cm:.2f} cm")
                    self.prev_diameter = diameter_cm
            else:
                self.get_logger().info("No orange detected.")
        else:
            self.get_logger().info("No ArUco marker detected.")

        msg = Float32()
        msg.data = diameter_cm
        self.publisher_.publish(msg)

        # Show frames
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)
        cv2.waitKey(10)

        # Optional: print FPS (debug only)
        now = time.time()
        fps = 1.0 / (now - self.last_frame_time)
        self.last_frame_time = now
        self.get_logger().debug(f"FPS: {fps:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = DiameterPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
