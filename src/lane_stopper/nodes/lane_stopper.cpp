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

bool LaneStopper::stopIntersection1(const autoware_msgs::DetectedObjectArray &input, const geometry_msgs::PoseStamped& mycarpose) {
    debug_publisher1.publish(input);
    debug_publisher2.publish(mycarpose);
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
                    if (dist(obj.pose, mycarpose.pose) < stop_distance_) {
                        // make_stop_waypoints();
                        ROS_ERROR("[%s] Stop at Intersection1.", __APP_NAME__);
                        return true;
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

void LaneStopper::SyncedDetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &input,
                                           const geometry_msgs::PoseStamped::ConstPtr& mycarpose) {
    bool flag;
    flag = LaneStopper::stopIntersection1(*input, *mycarpose);
    if (flag) {
        ROS_ERROR("[%s] Brake flag is true.", __APP_NAME__);
    }
    std_msgs::Bool msg;
    msg.data = flag;
    bool_publisher.publish(msg);
    // if (inIntersection1(*mycarpose)) ROS_ERROR("[%s] At Intersection1.", __APP_NAME__);
    // if (inIntersection2(*mycarpose)) ROS_ERROR("[%s] At Intersection2.", __APP_NAME__);
}

LaneStopper::LaneStopper() : node_handle_(), private_node_handle_("~"), tf_listener_() {
    private_node_handle_.param("stop_distance", stop_distance_, 20.0);
    private_node_handle_.param("objects_topic", objects_topic_, std::string("/detection/lidar_detector/objects"));
    private_node_handle_.param("current_pose_topic", pose_topic_, std::string("/current_pose"));
}

void LaneStopper::run() {
    objectArraySubscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_, objects_topic_, 1);
    poseSubscriber_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_handle_, pose_topic_, 1);
    synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *objectArraySubscriber_, *poseSubscriber_);
    synchronizer_->registerCallback(boost::bind(&LaneStopper::SyncedDetectionsCallback, this, _1, _2));

    debug_publisher1 = node_handle_.advertise<autoware_msgs::DetectedObjectArray>("/debug/lane_stopper/DetectedObjectArray", 1);
    debug_publisher2 = node_handle_.advertise<geometry_msgs::PoseStamped>("/debug/lane_stopper/PoseStamped", 1);
    bool_publisher = node_handle_.advertise<std_msgs::Bool>("/brake_flag", 10); // to avoid miss subscription
    
    ros::spin();
} 

int main(int argc, char **argv) {
  ros::init(argc, argv, "lane_stopper");
  LaneStopper node;
  node.run();
  return 0;
}
