 
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

    # # Original start point
    # msg.pose.pose.position.x = -8.3487071991 # 0
    # msg.pose.pose.position.y = 53.4187088013 # 0
    # msg.pose.pose.position.z = 0.0
    
    # Crossroad No.1 (After passing by a stopped car)
    rospy.loginfo("Intersection No.1")
    msg.pose.pose.position.x = -99.6277
    msg.pose.pose.position.y = -301.8317
    msg.pose.pose.position.z = -6.9193
    
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