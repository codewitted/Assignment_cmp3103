import math
import rclpy
from rclpy.node import Node
from rclpy import qos
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
import time

waypoint_route = [
    [1.3, 0.0, math.pi/2],
    [1.3, 1.3482, -3.0],   # FIXED (was pi)
    [0.0, 0.0, 0.0],
    [1.3, -1.3482, -math.pi/4],
]

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

def main():
    rclpy.init()

    node = Node("waypoint_publisher")

    publisher = node.create_publisher(
        PoseStamped,
        "/current_waypoint",
        qos_profile=qos.qos_profile_parameters
    )

    navigator = BasicNavigator()

    initial_pose = pose_from_xyquat(
        navigator.get_clock().now().to_msg()
    )

    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()

    waypoints = []
    for wp in waypoint_route:
        waypoints.append(
            pose_from_xytheta(
                navigator.get_clock().now().to_msg(),
                *wp
            )
        )

    navigator.followWaypoints(waypoints)

    i = 0

    while not navigator.isTaskComplete():
        i += 1

        feedback = navigator.getFeedback()

        if feedback is not None:
            current = feedback.current_waypoint

            if current is not None and current < len(waypoints):
                publisher.publish(waypoints[current])

                if i % 5 == 0:
                    print(f"Executing waypoint {current+1}/{len(waypoints)}")

        time.sleep(0.1)

    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        print("Task complete!")
    elif result == TaskResult.FAILED:
        print("Task failed!")
    elif result == TaskResult.CANCELED:
        print("Task canceled!")

    rclpy.shutdown()

if __name__ == '__main__':
    main()