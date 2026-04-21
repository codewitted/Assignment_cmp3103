#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
import math
import time
import tf2_ros
from tf_transformations import euler_from_quaternion


class CubePusher(Node):
    def __init__(self):
        super().__init__('cube_pusher')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Nav2 interface
        self.navigator = BasicNavigator()

        # Hard-coded map-frame positions
        self.behind_cube = (1.3441, 1.3482)
        self.red_zone = (-0.7396, 0.6963)

        # State machine
        self.state = "NAVIGATE_TO_CUBE"

        # Timer
        self.timer = self.create_timer(0.1, self.loop)

    def loop(self):
        if self.state == "NAVIGATE_TO_CUBE":
            self.navigate_to_pose(self.behind_cube)
            self.state = "WAIT_FOR_NAV"
            return

        if self.state == "WAIT_FOR_NAV":
            if self.navigator.isTaskComplete():
                self.get_logger().info("Reached behind-cube pose")
                self.state = "ROTATE_TO_GOAL"
            return

        if self.state == "ROTATE_TO_GOAL":
            self.rotate_towards(self.red_zone)
            self.state = "PUSH_FORWARD"
            self.push_start = time.time()
            return

        if self.state == "PUSH_FORWARD":
            # Push for 3 seconds (basic version)
            if time.time() - self.push_start < 3.0:
                twist = Twist()
                twist.linear.x = 0.15
                self.cmd_pub.publish(twist)
            else:
                self.state = "REVERSE"
                self.reverse_start = time.time()
            return

        if self.state == "REVERSE":
            if time.time() - self.reverse_start < 1.5:
                twist = Twist()
                twist.linear.x = -0.1
                self.cmd_pub.publish(twist)
            else:
                self.get_logger().info("Mission complete")
                self.state = "DONE"
            return

    # ---------------------------------------------------------
    # Helper: send Nav2 goal
    # ---------------------------------------------------------
    def navigate_to_pose(self, pos):
        x, y = pos
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y

        # Face the cube (approx)
        goal.pose.orientation.w = 1.0

        self.navigator.goToPose(goal)

    # ---------------------------------------------------------
    # Helper: rotate robot to face target point
    # ---------------------------------------------------------
    import tf2_ros
from tf_transformations import euler_from_quaternion

def rotate_towards(self, target):
    tx, ty = target

    # TF lookup
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Wait for transform
    try:
        trans = tf_buffer.lookup_transform(
            'map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
        )
    except:
        self.get_logger().warn("TF lookup failed, rotating blindly")
        twist = Twist()
        twist.angular.z = 0.4
        self.cmd_pub.publish(twist)
        time.sleep(1.0)
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        return

    # Extract robot pose
    rx = trans.transform.translation.x
    ry = trans.transform.translation.y

    q = trans.transform.rotation
    _, _, robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # Compute angle to target
    desired_yaw = math.atan2(ty - ry, tx - rx)
    yaw_error = desired_yaw - robot_yaw

    # Normalize
    yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

    # Rotate until aligned
    twist = Twist()
    twist.angular.z = 0.4 if yaw_error > 0 else -0.4
    self.cmd_pub.publish(twist)
    time.sleep(abs(yaw_error) / 0.4)

    twist.angular.z = 0.0



def main(args=None):
    rclpy.init(args=args)
    node = CubePusher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()