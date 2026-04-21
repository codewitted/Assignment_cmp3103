#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time


class ColourChaser(Node):
    def __init__(self):
        super().__init__('colour_chaser')

        # Subscribe to camera
        self.create_subscription(
            Image,
            '/limo/depth_camera_link/image_raw',
            self.camera_callback,
            1
        )

        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.br = CvBridge()

        # State variables
        self.last_seen_time = time.time()
        self.pushing_forward = False
        self.reversing = False

    def camera_callback(self, data):

        try:
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # ---------------------------------------------------------
        # RED CUBE DETECTION
        # ---------------------------------------------------------
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)

        mask_red = mask1 | mask2
        
        h, w = mask_red.shape
        mask_red[0:int(h * 0.4), :] = 0


        # Clean up noise
        kernel = np.ones((5, 5), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

        # Find contours
        contours, _ = cv2.findContours(
            mask_red.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE
        )

        twist = Twist()
        now = time.time()

        # ---------------------------------------------------------
        # STATE: PUSHING FORWARD AFTER LOSING THE CUBE
        # ---------------------------------------------------------
        if self.pushing_forward:
            if now - self.pushing_forward_start < 1.0:
                twist.linear.x = 0.15
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                print("Pushing forward blindly...")
                return
            else:
                # Switch to reversing
                self.pushing_forward = False
                self.reversing = True
                self.reverse_start = now

        # ---------------------------------------------------------
        # STATE: REVERSING TO CHECK IF CUBE IS STILL THERE
        # ---------------------------------------------------------
        if self.reversing:
            if now - self.reverse_start < 1.5:
                twist.linear.x = -0.1
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                print("Reversing to check...")
                return
            else:
                # Done reversing — check if cube is visible again
                self.reversing = False
                # If cube is visible, chasing will resume automatically
                # If not, search mode will resume automatically

        # ---------------------------------------------------------
        # NORMAL CHASE / SEARCH LOGIC
        # ---------------------------------------------------------
        if len(contours) > 0:
            # Cube visible → reset timers
            self.last_seen_time = now

            # Pick the largest red cube
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > 200:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), -1)

                    h, w, _ = cv_image.shape
                    center_x = w // 2
                    error = cx - center_x

                    # Chase mode
                    twist.angular.z = -float(error) / 300.0

                    if abs(error) < 50:
                        twist.linear.x = 0.15
                    else:
                        twist.linear.x = 0.0

                    self.cmd_pub.publish(twist)
                    return

        else:
            # Cube NOT visible
            time_since_seen = now - self.last_seen_time

            if time_since_seen < 0.2:
                # Just disappeared — start push-forward phase
                self.pushing_forward = True
                self.pushing_forward_start = now
                print("Cube vanished — initiating push-forward phase.")
                return

            # Otherwise → search mode
            twist.linear.x = 0.0
            twist.angular.z = 0.3
            self.cmd_pub.publish(twist)

        # Display windows
        cv2.imshow("Camera", cv2.resize(cv_image, (0, 0), fx=0.4, fy=0.4))
        cv2.imshow("Red Cube Mask", mask_red)
        cv2.waitKey(1)


def main(args=None):
    print("Starting colour_chaser.py")
    cv2.startWindowThread()

    rclpy.init(args=args)
    node = ColourChaser()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()