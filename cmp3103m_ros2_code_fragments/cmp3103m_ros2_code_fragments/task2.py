import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time
import threading


class ColourChaser(Node):
    def __init__(self):
        super().__init__('colour_chaser')

        self.create_subscription(
            Image,
            '/limo/depth_camera_link/image_raw',
            self.camera_callback,
            1
        )
        # Publisher for robot movements
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Convert between ROS to OpenCV
        self.br = CvBridge()

        # State variables
        self.pushing_forward = False
        self.reversing = False

        # Timing for detections
        self.last_arena_seen_time = 0.0 # Blindly push
        self.last_blue_seen_time = 0.0 # Reverse after push

        self.prev_ang = 0.0

        # For non-blocking display
        self.latest_frame = None
        self.latest_masks = None
        self.start_display_thread()

    # Checks if the blue object is over lapping the red region
    def is_touching_red(self, blue_mask, red_mask):
        overlap = cv2.bitwise_and(blue_mask, red_mask)
        return np.any(overlap > 0)

    # Displays camera and masks
    def start_display_thread(self):
        thread = threading.Thread(target=self.display_loop, daemon=True)
        thread.start()

    # Displays camera and masks continously
    def display_loop(self):
        while True:
            if self.latest_frame is not None and self.latest_masks is not None:
                frame = cv2.resize(self.latest_frame, (0, 0), fx=0.4, fy=0.4)
                mask_red, mask_brown, mask_blue = self.latest_masks

                cv2.imshow("Camera", frame)
                cv2.imshow("Red Arena Mask", mask_red)
                cv2.imshow("Brown Mask", mask_brown)
                cv2.imshow("Blue Mask", mask_blue)
                cv2.waitKey(1)
            time.sleep(0.01)

    # Perception and control
    def camera_callback(self, data):

        try:
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            return

        # Converts to HSV segmentation
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Red area mask
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)

        mask_red = mask1 | mask2

        h, w = mask_red.shape

        # Ignore the red on the wall (top 15% of the camera frame)
        mask_red[0:int(h * 0.15), :] = 0

        # Strengthen red border detection using morphology
        kernel = np.ones((5, 5), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        mask_red = cv2.dilate(mask_red, kernel, iterations=2)

        # Dilated red mask for touching detection
        red_dilated = cv2.dilate(mask_red, np.ones((7, 7), np.uint8))
        
        # Find red contours
        red_contours, _ = cv2.findContours(
            mask_red.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # Brown cube mask
        lower_brown = np.array([10, 100, 20])
        upper_brown = np.array([25, 255, 200])
        mask_brown = cv2.inRange(hsv_img, lower_brown, upper_brown)

        # Blue cube mask
        lower_blue = np.array([90, 80, 40])
        upper_blue = np.array([130, 255, 255])
        mask_blue = cv2.inRange(hsv_img, lower_blue, upper_blue)

        twist = Twist()
        now = time.time()

        # Blind push state
        if self.pushing_forward:
            if now - self.pushing_forward_start < 2.0:
                twist.linear.x = 0.15
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
            else:
                # Switch to reverse after pushing
                self.pushing_forward = False
                self.reversing = True
                self.reverse_start = now

            self.latest_frame = cv_image.copy()
            self.latest_masks = (mask_red.copy(), mask_brown.copy(), mask_blue.copy())
            return

        # Reverse State
        if self.reversing:
            if now - self.reverse_start < 1.5:
                twist.linear.x = -0.1
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
            else:
                self.reversing = False

            self.latest_frame = cv_image.copy()
            self.latest_masks = (mask_red.copy(), mask_brown.copy(), mask_blue.copy())
            return

        
        target_found = False

        # Detect red patch
        if len(red_contours) > 0:
            self.last_arena_seen_time = now

             # Use largest red contour as arena
            arena_contour = max(red_contours, key=cv2.contourArea)
            x, y, rw, rh = cv2.boundingRect(arena_contour)

            cv2.rectangle(cv_image, (x, y), (x+rw, y+rh), (0, 0, 255), 2)

            # Expand region of interest (ROI)
            pad = 10
            x0 = max(0, x - pad)
            y0 = max(0, y - pad)
            x1 = min(w, x + rw + pad)
            y1 = min(h, y + rh + pad)

            roi_brown = mask_brown[y0:y1, x0:x1]
            roi_blue = mask_blue[y0:y1, x0:x1]
            roi_red_dilated = red_dilated[y0:y1, x0:x1]

            brown_contours, _ = cv2.findContours(
                roi_brown.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            blue_contours, _ = cv2.findContours(
                roi_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            # Target blue cubes if touching red
            if len(blue_contours) > 0:
                bc = max(blue_contours, key=cv2.contourArea)
                area = cv2.contourArea(bc)

                if area > 80:
                    M = cv2.moments(bc)
                    if M["m00"] != 0:
                        bx = int(M["m10"] / M["m00"]) + x0
                        by = int(M["m01"] / M["m00"]) + y0

                        # Check touching condition
                        if not self.is_touching_red(roi_blue, roi_red_dilated):
                            pass
                        else:
                            cv2.circle(cv_image, (bx, by), 10, (255, 0, 0), -1)

                            self.last_blue_seen_time = now
                            target_found = True

                            center_x = w // 2
                            error = bx - center_x

                            dead_zone = 60
                            
                            # Align to target
                            if abs(error) > dead_zone:
                                # Rotate to target centre
                                twist.angular.z = -float(error) / 300.0
                                twist.linear.x = 0.0
                            else:
                                # Move forward when aligned
                                twist.angular.z = 0.0
                                twist.linear.x = 0.12

                            # Brown avoidance
                            for bc2 in brown_contours:
                                bx2, by2, bw2, bh2 = cv2.boundingRect(bc2)
                                cx2 = bx2 + bw2 // 2
                                cy2 = by2 + bh2 // 2
                                cx2 += x0
                                cy2 += y0

                                if abs(cx2 - center_x) < 60 and cy2 > h * 0.5:
                                    twist.angular.z += 0.4
                                    twist.linear.x = 0.05

                            # Smooth angular velocity
                            alpha = 0.3
                            twist.angular.z = alpha * twist.angular.z + (1 - alpha) * self.prev_ang
                            self.prev_ang = twist.angular.z

                            self.cmd_pub.publish(twist)

        # Search for targets
        if not target_found:
            if now - self.last_blue_seen_time < 0.2:
                self.pushing_forward = True
                self.pushing_forward_start = now
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.3
                self.cmd_pub.publish(twist)

        # Store for display thread
        self.latest_frame = cv_image.copy()
        self.latest_masks = (mask_red.copy(), mask_brown.copy(), mask_blue.copy())


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