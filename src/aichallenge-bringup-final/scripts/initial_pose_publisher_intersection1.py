#!/usr/bin/env python
# coding: utf-8

from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
import time
import math

# const static
_original_yaw = -1.8429

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

    # # Original start point
    # msg.pose.pose.position.x = -8.3487071991 # 0
    # msg.pose.pose.position.y = 53.4187088013 # 0
    # msg.pose.pose.position.z = 0.0

    # Crossroad No.1 (After passing by a stopped car)
    rospy.loginfo("Intersection No.1")
    msg.pose.pose.position.x = -97.7784
    msg.pose.pose.position.y = -338.4901  # <- Caution!!, NOT Y, BUT Z
    msg.pose.pose.position.z = -7.4541  # <- Caution!!, NOT Z, BUT Y

    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = -0.791386926148  # az * sin(θ/2)
    msg.pose.pose.orientation.w = 0.611315575723 # cos(θ/2) 
    # θ/2 = -0.9130, θ = -1.8261
    print('original : ', -0.791386926148, 0.611315575723)
    z_, w_ = yaw2z_and_w(-0.2357)
    print('debug : ', z_, w_)
    msg.pose.pose.orientation.z = z_
    msg.pose.pose.orientation.w = w_

    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.06853892326654787
    time.sleep(1)
    publisher.publish(msg)
    rospy.spin()
