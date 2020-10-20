import message_filters
import rospy
from autoware_msgs.msg import VehicleCmd
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped
import math

_VEL_LIMIT = 28 * (10 ** 3) / (3600) # m/s

class SimpleCtrl:
    def __init__(self):
        rospy.init_node('listener', anonymous=True, disable_signals = True)
        self.vel_norm = 0
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0

        self.rate = rospy.Rate(200) # 10hz

        self.cmd_msg = VehicleCmd()
        self.cmd_msg.gear = 64  # Drive
        # self.cmd_msg = ControlCommandStamped()

        # subscribers
        rospy.Subscriber("/current_velocity", TwistStamped, self.cb_cur_vel)

        # publishers
        self.pub_accel = rospy.Publisher('/vehicle_cmd', VehicleCmd, queue_size=1)
        # self.pub_accel = rospy.Publisher('/ctrl_cmd', ControlCommandStamped, queue_size=1)
    
    def cb_cur_vel(self, twist_stamped_msgs):
        if twist_stamped_msgs:
            self.vel_x = twist_stamped_msgs.twist.linear.x
            self.vel_y = twist_stamped_msgs.twist.linear.y
            self.vel_z = twist_stamped_msgs.twist.linear.z
            # print(self.vel_x)
            self.vel_norm = math.sqrt(self.vel_x**2 + self.vel_y**2 + self.vel_z**2)

        if self.vel_norm < _VEL_LIMIT:
            rospy.loginfo(" %s m/s, I KKE EE!!" % self.vel_norm)
            self.cmd_msg.ctrl_cmd.linear_acceleration =  -1
            # self.cmd_msg.cmd.linear_acceleration =  1
            self.pub_accel.publish(self.cmd_msg)
            pass
        else:
            rospy.logwarn("%s m/s TO MA RE !!" % self.vel_norm)
            self.cmd_msg.ctrl_cmd.linear_acceleration =  -1
            # self.cmd_msg.cmd.linear_acceleration =  -1
            self.pub_accel.publish(self.cmd_msg)
            pass
    
    def run(self):
        while (1):
            rospy.loginfo(" %s FREEZE !!" % self.vel_norm)
            self.cmd_msg.ctrl_cmd.linear_acceleration =  -1
            self.pub_accel.publish(self.cmd_msg)
            self.rate.sleep()
            pass

        # rospy.spin()

        
if __name__ == '__main__':
    ctrl = SimpleCtrl()
    ctrl.run()