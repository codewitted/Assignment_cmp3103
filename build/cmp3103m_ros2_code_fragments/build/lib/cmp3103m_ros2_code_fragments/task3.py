import math
import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf_transformations import quaternion_from_euler

# Camera imports
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# TF2 imports (robot position tracking)
from tf2_ros import Buffer, TransformListener


# Camera
class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        self.bridge = CvBridge()

        # Store detections
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
            # convert ROS image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Colour filtering
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # Noise reduction
            kernel = np.ones((5, 5), np.uint8)

            # Create colour mask and clean noise
            mask_small = cv2.inRange(hsv, (5, 80, 40), (20, 255, 104))
            mask_small = cv2.morphologyEx(mask_small, cv2.MORPH_OPEN, kernel)
            mask_small = cv2.morphologyEx(mask_small, cv2.MORPH_CLOSE, kernel)

            # Find contours
            contours, _ = cv2.findContours(mask_small, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Reset detections for each frame
            self.small_detected = False
            self.small_center_x = None

            for c in contours:
                if cv2.contourArea(c) < 80:
                    continue
                
                # Create bounding box around the object
                x, y, w, h = cv2.boundingRect(c)
                # Calculate the object's centre
                cx = x + w // 2

                # Update detection
                self.small_detected = True
                self.small_center_x = cx

                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # Show camera
            cv2.imshow("Camera", frame)
            cv2.imshow("Small Mask", mask_small)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")


# Pose lookup
def get_robot_pose(tf_buffer):
    try:
        # Returns the robot's current position in the map frame
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


def pose_from_xyquat(timestamp, x=0.0, y=0.0, pz=0.0, pw=1.0):
    # Uses quaternion values to create PoseStamped
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

# First 3 wapoints to position behind the object
waypoint_route = [
    [1.0, 0.0, math.pi/4],
    # [1.1, 1.0, math.pi/2],
    [1.1, 1.45, math.pi/2],
]
# Last 4 waypoints to push the object to the red square
waypoint_route_after = [
    [0.9, 1.2, -3*math.pi/4],
    [0.5, 0.6, math.pi],
    [-0.5, 0.8, math.pi],
    [-0.74, 0.9, math.pi],
]

def main():
    rclpy.init()

    camera_node = CameraNode()
    # Publisher for robot movement
    cmd_pub = camera_node.create_publisher(Twist, "/cmd_vel", 10)

    navigator = BasicNavigator()

    # Position tracking
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, camera_node)

    # Set the initial pose
    initial_pose = pose_from_xyquat(navigator.get_clock().now().to_msg())
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()

    # Waypoints 1–3
    waypoints = []
    for wp in waypoint_route:
        waypoints.append(
            pose_from_xytheta(navigator.get_clock().now().to_msg(), *wp)
        )

    # Start Navigation
    navigator.followWaypoints(waypoints)

    # Wait until navigation finishes
    while rclpy.ok() and not navigator.isTaskComplete():
        rclpy.spin_once(camera_node, timeout_sec=0.01)

    print("\nReached aligning waypoint\n")

    # Align the object to the desired part of the camera
    target_x = 320

    while rclpy.ok():
        rclpy.spin_once(camera_node, timeout_sec=0.01)
        # Rotate until the object is seen
        if not camera_node.small_detected:
            twist = Twist()
            twist.angular.z = 0.3
            cmd_pub.publish(twist)
            continue
        
        # horizontal error
        error = camera_node.small_center_x - target_x
        
        if abs(error) < 15:
            break
        # Rotate towards the object
        twist = Twist()
        twist.angular.z = -0.006 * error
        cmd_pub.publish(twist)

    cmd_pub.publish(Twist())
    time.sleep(0.3)

    print("\nPushing object left\n")

    lost_time = None

    # Push until desired location is reached
    while rclpy.ok():

        rclpy.spin_once(camera_node, timeout_sec=0.01)

        # Check robot's position
        pose_msg = get_robot_pose(tf_buffer)
        if pose_msg:
            px = pose_msg.pose.position.x
            py = pose_msg.pose.position.y
            # If the robot has reached the desired y-pose, turn left
            # and continue waypoints (4-7)
            if py < 0.9:
                print("\nDesired position reached\n")

                start_time = time.time()

                # Turn left for 5 seconds so the cube is not pushed
                while time.time() - start_time < 5.0 and rclpy.ok():
                    rclpy.spin_once(camera_node, timeout_sec=0.01)

                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.4   # LEFT TURN
                    cmd_pub.publish(twist)

                # stop robot
                cmd_pub.publish(Twist())
                time.sleep(0.2)

                print("\nPushing cube to red square\n")

                # Execute second set of waypoints
                waypoints_after = [
                    pose_from_xytheta(
                        navigator.get_clock().now().to_msg(),
                        *wp
                    )
                    for wp in waypoint_route_after
                ]

                navigator.followWaypoints(waypoints_after)

                camera_node.destroy_node()
                rclpy.shutdown()

                break

        # adjust position to push left
        if camera_node.small_detected:

            lost_time = None

            # Camera offset
            target_x = 320   # tune 200–260

            error = camera_node.small_center_x - target_x

            twist = Twist()

            # Move forward slowly
            twist.linear.x = 0.05

            # Avoid getting stuck
            ang = -0.02 * error
            ang = max(min(ang, 1.2), -1.2)

            twist.angular.z = ang

            cmd_pub.publish(twist)
            continue

        # If object is no longer in camera
        if lost_time is None:
            lost_time = time.time()

        if time.time() - lost_time > 2.5:

            while rclpy.ok():
                rclpy.spin_once(camera_node, timeout_sec=0.01)

                if camera_node.small_detected:
                    break

                twist = Twist()
                # if object is lost for too long, rotate to find it (back against a wall)
                if time.time() - lost_time > 10.0:
                    print("block completely lost")
                    twist.linear.x = 0.0
                    twist.angular.z = -0.35
                
                else:
                    twist.linear.x = -0.1
                    twist.angular.z = -0.1
                cmd_pub.publish(twist)

        # Push block blindly
        twist = Twist()
        twist.linear.x = 0.05
        twist.angular.z = 0.35
        cmd_pub.publish(twist)

if __name__ == '__main__':
    main()

# https://github.com/LCAS/ROB2002 - reference for waypoint movement