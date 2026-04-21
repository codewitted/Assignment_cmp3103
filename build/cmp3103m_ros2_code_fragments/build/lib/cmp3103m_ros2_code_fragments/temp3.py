#! /usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf_transformations import quaternion_from_euler

# CAMERA IMPORTS
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# TF2 IMPORTS
from tf2_ros import Buffer, TransformListener


# ---------------------------
# CAMERA NODE
# ---------------------------
class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        self.bridge = CvBridge()

        self.small_center_x = None
        self.small_detected = False

        self.create_subscription(
            Image,
            "/limo/depth_camera_link/image_raw",
            self.camera_callback,
            10
        )

    def camera_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            kernel = np.ones((5, 5), np.uint8)

            mask_small = cv2.inRange(hsv, (5, 80, 40), (20, 255, 104))
            mask_small = cv2.morphologyEx(mask_small, cv2.MORPH_OPEN, kernel)
            mask_small = cv2.morphologyEx(mask_small, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask_small, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            self.small_detected = False
            self.small_center_x = None

            for c in contours:
                if cv2.contourArea(c) < 80:
                    continue

                x, y, w, h = cv2.boundingRect(c)
                cx = x + w // 2

                self.small_detected = True
                self.small_center_x = cx

                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

            cv2.imshow("Camera", frame)
            cv2.imshow("Small Mask", mask_small)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")


# ---------------------------
# TF2 POSE LOOKUP
# ---------------------------
def get_robot_pose(tf_buffer):
    try:
        trans = tf_buffer.lookup_transform(
            'map',
            'base_link',
            rclpy.time.Time()
        )

        pose = PoseStamped()
        pose.header = trans.header
        pose.pose.position.x = trans.transform.translation.x
        pose.pose.position.y = trans.transform.translation.y
        pose.pose.orientation = trans.transform.rotation

        return pose

    except Exception:
        return None


# ---------------------------
# WAYPOINT HELPERS
# ---------------------------
def pose_from_xyquat(timestamp, x=0.0, y=0.0, pz=0.0, pw=1.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = timestamp
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = pz
    pose.pose.orientation.w = pw
    return pose


def pose_from_xytheta(timestamp, x=0.0, y=0.0, theta=0.0):
    q = quaternion_from_euler(0, 0, theta)
    return pose_from_xyquat(timestamp, x, y, q[2], q[3])


# ---------------------------
# WAYPOINT ROUTES
# ---------------------------
waypoint_route = [
    [1.0, 0.0, math.pi/2],
    [1.3, 1.0, math.pi/2],
    [1.2, 1.45, math.pi/2],
]

waypoint_route_after = [
    [0.0, 0.0, 0.0],
    [1.3, 0.8, math.pi],
    [-0.74, 0.7, math.pi],
]


# ---------------------------
# MAIN
# ---------------------------
def main():
    rclpy.init()

    camera_node = CameraNode()
    cmd_pub = camera_node.create_publisher(Twist, "/cmd_vel", 10)

    navigator = BasicNavigator()

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, camera_node)

    initial_pose = pose_from_xyquat(navigator.get_clock().now().to_msg())
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()

    # Waypoints 1–3
    waypoints = []
    for wp in waypoint_route:
        waypoints.append(
            pose_from_xytheta(navigator.get_clock().now().to_msg(), *wp)
        )

    waypoint4 = pose_from_xytheta(
        navigator.get_clock().now().to_msg(),
        *waypoint_route_after[0]
    )

    navigator.followWaypoints(waypoints)

    while rclpy.ok() and not navigator.isTaskComplete():
        rclpy.spin_once(camera_node, timeout_sec=0.01)

    print("\nReached aligning waypoint\n")

    # ---------------------------
    # INITIAL ALIGN (CENTER)
    # ---------------------------
    target_x = 320

    while rclpy.ok():
        rclpy.spin_once(camera_node, timeout_sec=0.01)

        if not camera_node.small_detected:
            twist = Twist()
            twist.angular.z = 0.3
            cmd_pub.publish(twist)
            continue

        error = camera_node.small_center_x - target_x

        if abs(error) < 15:
            break

        twist = Twist()
        twist.angular.z = -0.006 * error
        cmd_pub.publish(twist)

    cmd_pub.publish(Twist())
    time.sleep(0.3)

    print("\nPushing with sideways bias...\n")

    lost_time = None

    while rclpy.ok():

        rclpy.spin_once(camera_node, timeout_sec=0.01)

        # ---------------------------
        # POSITION CHECK
        # ---------------------------
        pose_msg = get_robot_pose(tf_buffer)
        if pose_msg:
            px = pose_msg.pose.position.x
            py = pose_msg.pose.position.y

            if py < 0.85:
                print("\nReversing before waypoint 4...\n")

                start_time = time.time()

                while time.time() - start_time < 1.0 and rclpy.ok():
                    rclpy.spin_once(camera_node, timeout_sec=0.01)
                    twist = Twist()
                    twist.linear.x = -0.1   # reverse
                    twist.angular.z = 0.0
                    cmd_pub.publish(twist)

                # stop robot
                cmd_pub.publish(Twist())
                time.sleep(0.2)

                print("\nGoing to waypoint 4\n")
                navigator.goToPose(waypoint4)
                break

        # ---------------------------
        # 🔥 SIDEWAYS PUSH CONTROL
        # ---------------------------
        if camera_node.small_detected:

            lost_time = None

            # 🔥 OFFSET TARGET (THIS CREATES SIDEWAYS FORCE)
            target_x = 220   # tune 200–260

            error = camera_node.small_center_x - target_x

            twist = Twist()

            # slower forward → allows turning to matter
            twist.linear.x = 0.07

            # stronger turning
            ang = -0.012 * error
            ang = max(min(ang, 0.8), -0.8)

            twist.angular.z = ang

            cmd_pub.publish(twist)
            continue

        # ---------------------------
        # LOST LOGIC
        # ---------------------------
        if lost_time is None:
            lost_time = time.time()

        if time.time() - lost_time > 2.0:

            while rclpy.ok():
                rclpy.spin_once(camera_node, timeout_sec=0.01)

                if camera_node.small_detected:
                    break

                twist = Twist()
                if px <= 0.5 or py >= 1.3:
                    twist.linear.x = -0.1
                    twist.angular.z = 0.1
                else:
                    twist.linear.x = -0.1
                    twist.angular.z = -0.1
                cmd_pub.publish(twist)

        # blind motion
        twist = Twist()
        twist.linear.x = 0.05
        twist.angular.z = 0.3
        cmd_pub.publish(twist)

    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()