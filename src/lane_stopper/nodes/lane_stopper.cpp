#include "lane_stopper.h"
#define __APP_NAME__ "lane_stopper"

// y0
// y = -0.2582730920132578 x + -361.2845319741398
// y1
// y = -0.26430773408498043 x + -355.9439251504078
// y2 Intersection1
// y = -0.24750019525741684 x + -347.22055328879355
// y3 Intersection2
// y = 3.9263219260739675 x + -160.03610323652867
// y4
// y = 3.7920438197948 x + -212.38461245157148


bool LaneStopper::objectInIncomingLane(const autoware_msgs::DetectedObject &object) {
    double x, y;
    x = object.pose.position.x;
    y = object.pose.position.y; 
    if ((a[0] * x + b[0] < y) && (y < a[1] * x + b[1])) return true;
    else return false;
}

bool LaneStopper::objectIncomingIntersection2(const autoware_msgs::DetectedObject &object) {
    double x, y;
    x = object.pose.position.x;
    y = object.pose.position.y; 
    if (y < a[4] * x + b[4]) return true;
    else return false;
}

bool LaneStopper::objectIncomingIntersection1(const autoware_msgs::DetectedObject &object) {
    double x, y;
    x = object.pose.position.x;
    y = object.pose.position.y; 
    if (y < a[5] * x + b[5]) return true;
    else return false;
}


bool LaneStopper::inIntersection1(const geometry_msgs::PoseStamped& msg) {
    geometry_msgs::Point p = msg.pose.position;
    if ((a[1] * p.x + b[1] < p.y) && (p.y < a[2] * p.x + b[2])) return true;
    else return false;
}

bool LaneStopper::inIntersection2(const geometry_msgs::PoseStamped& msg) {
    geometry_msgs::Point p = msg.pose.position;
    if ((p.y < a[0] * p.x + b[0]) && (p.y < a[3] * p.x + b[3])) return true;
    else return false;
}

double LaneStopper::dist(const geometry_msgs::Pose p, const geometry_msgs::Pose q) {
    return std::hypot(p.position.x - q.position.x, p.position.y - q.position.y);
}

bool LaneStopper::stopIntersection(const autoware_msgs::DetectedObjectArray &input, const geometry_msgs::PoseStamped& mycarpose) {
    try {
        tf::StampedTransform convertMatrix;
        tf_listener_.waitForTransform("map", input.header.frame_id, ros::Time(0), ros::Duration(1.0));
        tf_listener_.lookupTransform("map", input.header.frame_id, ros::Time(0), convertMatrix);

        for (size_t i = 0; i < input.objects.size(); i++) {
            autoware_msgs::DetectedObject obj = input.objects[i];
            tf::Transform tmp;
            tf::poseMsgToTF(obj.pose, tmp);
            tf::poseTFToMsg(convertMatrix * tmp, obj.pose);
            // 対向車線に車がいたら　
            if (objectInIncomingLane(obj)) {
                ROS_INFO("[%s] Object in the incomming lane.", __APP_NAME__);
                // 自車が交差点1にいるとき
                if (inIntersection1(mycarpose)) {
                    if (objectIncomingIntersection1(obj)) {
                        if (dist(obj.pose, mycarpose.pose) < stop_distance_) {
                            // make_stop_waypoints();
                            ROS_ERROR("[%s] Stop at Intersection1.", __APP_NAME__);
                            return true;
                        }
                    }
                }
                // 自車が交差点2にいるとき
                else if (inIntersection2(mycarpose)) {
                    // 対向車線上の避けるべきいちに対向車がきていたら
                    if (objectIncomingIntersection2(obj)) {
                        if (dist(obj.pose, mycarpose.pose) < stop_distance_) {
                            // make_stop_waypoints();
                            ROS_ERROR("[%s] Stop at Intersection2.", __APP_NAME__);
                            return true;
                        }
                    }
                }
            }
        }
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("[%s] %s", __APP_NAME__, ex.what());
        return false;
    }
    return false;
}

void LaneStopper::syncedDetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &input,
                                           const geometry_msgs::PoseStamped::ConstPtr& mycarpose) {
    brake_flag_ = LaneStopper::stopIntersection(*input, *mycarpose);
    if (brake_flag_) {
        ROS_ERROR("[%s] Brake flag is true.", __APP_NAME__);
    }
    std_msgs::Bool msg;
    msg.data = brake_flag_;
    bool_publisher.publish(msg);
    // if (inIntersection1(*mycarpose)) ROS_ERROR("[%s] At Intersection1.", __APP_NAME__);
    // if (inIntersection2(*mycarpose)) ROS_ERROR("[%s] At Intersection2.", __APP_NAME__);
}

void LaneStopper::reset_vehicle_cmd_msg()
{
  twist_gate_msg_.twist_cmd.twist.linear.x = 0;
  twist_gate_msg_.twist_cmd.twist.angular.z = 0;
  twist_gate_msg_.mode = 0;
  twist_gate_msg_.gear = 0;
  twist_gate_msg_.lamp_cmd.l = 0;
  twist_gate_msg_.lamp_cmd.r = 0;
  twist_gate_msg_.accel_cmd.accel = 0;
  twist_gate_msg_.brake_cmd.brake = 0;
  twist_gate_msg_.steer_cmd.steer = 0;
  twist_gate_msg_.ctrl_cmd.linear_velocity = 0;
  twist_gate_msg_.ctrl_cmd.steering_angle = 0;
  twist_gate_msg_.emergency = 0;
}

void LaneStopper::vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& input_msg) {
    // twist_gate_msg_ = *input_msg;

    LaneStopper::reset_vehicle_cmd_msg();
    double accel, steer;
    twist_gate_msg_.gear = 64;
    steer = - (input_msg->ctrl_cmd.steering_angle / PI * 180.0) / 39.4; 
    accel = input_msg->ctrl_cmd.linear_acceleration / accel_divide_gain_;
    
    accel = std::max(deccel_limit_, std::min(accel, accel_limit_));
    filtered_accel_ = lowpass_gain_ * filtered_accel_ + (1 - lowpass_gain_) * accel;
    filtered_steer_ = lowpass_gain_steer_ * filtered_steer_ + (1 - lowpass_gain_steer_) * steer;
    
    twist_gate_msg_.ctrl_cmd.linear_acceleration = filtered_accel_;
    twist_gate_msg_.ctrl_cmd.steering_angle = filtered_steer_;
    
    if (brake_flag_) {
        twist_gate_msg_.ctrl_cmd.linear_velocity = -10.0;
        twist_gate_msg_.ctrl_cmd.linear_acceleration = -10.0;
    }
    vehicle_cmd_pub.publish(twist_gate_msg_);
}

LaneStopper::LaneStopper() : node_handle_(), private_node_handle_("~"), tf_listener_(), brake_flag_(false), filtered_accel_(0.0), filtered_steer_(0.0) {
    private_node_handle_.param("stop_distance", stop_distance_, 20.0);
    private_node_handle_.param("objects_topic", objects_topic_, std::string("/detection/lidar_detector/objects"));
    private_node_handle_.param("current_pose_topic", pose_topic_, std::string("/current_pose"));
    private_node_handle_.param("accel_divide_gain", accel_divide_gain_, 20.0);
    private_node_handle_.param("lowpass_gain", lowpass_gain_, 0.0);
    private_node_handle_.param("accel_limit", accel_limit_, 1.0);
    private_node_handle_.param("deccel_limit", deccel_limit_, -1.0);
    private_node_handle_.param("lowpass_gain_steer", lowpass_gain_steer_, 0.0);
}

void LaneStopper::run() {
    objectArraySubscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_, objects_topic_, 1);
    poseSubscriber_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_handle_, pose_topic_, 1);
    synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *objectArraySubscriber_, *poseSubscriber_);
    synchronizer_->registerCallback(boost::bind(&LaneStopper::syncedDetectionsCallback, this, _1, _2));

    vehicle_cmd_sub_ = node_handle_.subscribe("vehicle_cmd_pre", 1, &LaneStopper::vehicleCmdCallback, this);

    bool_publisher = node_handle_.advertise<std_msgs::Bool>("/brake_flag", 10); // to avoid miss subscription
    vehicle_cmd_pub = node_handle_.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 1, true);

    ros::spin();
} 

int main(int argc, char **argv) {
  ros::init(argc, argv, "lane_stopper");
  LaneStopper node;
  node.run();
  return 0;
}
