#!/usr/bin/env python
# coding: utf-8

from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
import time
import math

# Set start position
x_y_z_yaw = [-56.4859, -348.4951, -7.5593, -0.2357]

def yaw2z_and_w(saved_pont_yaw):
    """[summary]
    Az axis rotation

    Args:
        saved_pont_yaw ([type]): [description]

    Returns:
        [type]: [description]
    """
    half_theta = saved_pont_yaw / 2.
    z = math.sin(half_theta)
    w = math.cos(half_theta)
    return z, w


if __name__ == "__main__":
    # Initialize publisher node
    rospy.init_node("initial_pose_pub", anonymous=True)
    publisher = rospy.Publisher(
        "initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
    # Run publisher
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"

    # Crossroad No.1 (After passing by a stopped car)
    rospy.loginfo("Intersection No.1")
    msg.pose.pose.position.x = x_y_z_yaw[0]
    msg.pose.pose.position.y = x_y_z_yaw[1]
    msg.pose.pose.position.z = x_y_z_yaw[2]

    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    z_, w_ = yaw2z_and_w(x_y_z_yaw[3])  # cal Quaternion
    msg.pose.pose.orientation.z = z_
    msg.pose.pose.orientation.w = w_

    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.06853892326654787
    time.sleep(1)
    publisher.publish(msg)
    rospy.spin()
