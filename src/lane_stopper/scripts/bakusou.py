#!/usr/bin/env python
# coding: utf-8

import message_filters
import rospy
from autoware_msgs.msg import VehicleCmd
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped
import math


class BrakingCtrl:
    def __init__(self):
        rospy.init_node('braking_node', anonymous=True, disable_signals=True)
        # init variables
        self.vel_norm = 0
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0
        self.set_bake = True

        # init const values
        # m/s, used for detect velocity limit
        self.vel_limit = 28 * (10 ** 3) / (3600)
        self.rate = rospy.Rate(100)  # 100hz
        self.cmd_msg = VehicleCmd()
        self.cmd_msg.gear = 64  # Drive

        # subscribers
        rospy.Subscriber("/brake_flag", Bool, self.cb_barke_listen, queue_size=10)
        # rospy.Subscriber("/current_velocity", TwistStamped, self.cb_cur_vel)

        # publishers
        self.pub_accel = rospy.Publisher('vehicle_cmd', VehicleCmd, queue_size=1)

    def cb_barke_listen(self, msg):
        if msg is not None:
            self.set_bake = True if msg.data else False

    def cb_cur_vel(self, twist_stamped_msgs):
        """[summary]
        Not used

        Args:
            twist_stamped_msgs ([type]): [description]
        """
        if twist_stamped_msgs:
            self.vel_x = twist_stamped_msgs.twist.linear.x
            self.vel_y = twist_stamped_msgs.twist.linear.y
            self.vel_z = twist_stamped_msgs.twist.linear.z
            self.vel_norm = math.sqrt(
                self.vel_x**2 + self.vel_y**2 + self.vel_z**2)

        if self.vel_norm > self.vel_limit:
            km_p_h = self.vel_norm * 3600. / 1000
            rospy.loginfo(" %s is MA ZU I DE SU YO !" % km_p_h)
            self.cmd_msg.ctrl_cmd.linear_acceleration = -1
            self.pub_accel.publish(self.cmd_msg)
            pass

    def run(self):
        rospy.loginfo("[BrakeCtrl] start")
        while 1:
            if self.set_bake:
                rospy.loginfo("[BrakeCtrl] I KI SU GI II !!")
                self.cmd_msg.ctrl_cmd.linear_acceleration = 1
                self.pub_accel.publish(self.cmd_msg)
            else:
                # rospy.loginfo(" uhoku ")
                pass
            self.rate.sleep()
            pass
        rospy.loginfo("[BrakeCtrl] end")



if __name__ == '__main__':
    ctrl = BrakingCtrl()
    ctrl.run()
