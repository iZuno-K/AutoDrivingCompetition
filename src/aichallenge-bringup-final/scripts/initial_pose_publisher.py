#!/usr/bin/env python
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
import time

if __name__ == "__main__":
    # Initialize publisher node
    rospy.init_node("initial_pose_pub", anonymous=True)
    publisher = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
    # Run publisher
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    
    # offset_forward=0, offset_right=0, up_weight=0
    # msg.pose.pose.position.x = -8.3487071991 # 0
    # msg.pose.pose.position.y = 53.4187088013 # 0
    # msg.pose.pose.position.z = 0.0

    # offset_forward=50, offset_right=0, up_weight=0
    msg.pose.pose.position.x = -21.8220252990723 # Vector.x
    msg.pose.pose.position.z = -2.02170538902283 # Vector.y
    msg.pose.pose.position.y = 2.0040488243103 # Vector.z


    # # offset_forward=100, offset_right=5, up_weight=0
    # msg.pose.pose.position.x = -39.4476509094238
    # msg.pose.pose.position.y = -2.75920009613037
    # msg.pose.pose.position.z = -45.0526962280273

    
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = -0.791386926148
    msg.pose.pose.orientation.w = 0.611315575723
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.06853892326654787
    time.sleep(1)
    publisher.publish(msg)
    rospy.spin()