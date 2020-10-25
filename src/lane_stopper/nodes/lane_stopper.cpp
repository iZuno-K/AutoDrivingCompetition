#include "lane_stopper.h"
#define __APP_NAME__ "lane_stopper"


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
}

void LaneStopper::reset_vehicle_cmd_msg() {
  vehicle_cmd_msg_.twist_cmd.twist.linear.x = 0;
  vehicle_cmd_msg_.twist_cmd.twist.angular.z = 0;
  vehicle_cmd_msg_.mode = 0;
  vehicle_cmd_msg_.gear = 0;
  vehicle_cmd_msg_.lamp_cmd.l = 0;
  vehicle_cmd_msg_.lamp_cmd.r = 0;
  vehicle_cmd_msg_.accel_cmd.accel = 0;
  vehicle_cmd_msg_.brake_cmd.brake = 0;
  vehicle_cmd_msg_.steer_cmd.steer = 0;
  vehicle_cmd_msg_.ctrl_cmd.linear_velocity = 0;
  vehicle_cmd_msg_.ctrl_cmd.steering_angle = 0;
  vehicle_cmd_msg_.emergency = 0;
}



void LaneStopper::CtrlCmdCallback(const autoware_msgs::ControlCommandStampedConstPtr& input_msg) {
    health_checker_.NODE_ACTIVATE();
    autoware_msgs::ControlCommandStamped ccs;
    checkCtrl(*input_msg);
    ccs = lateralLimitCtrl(*input_msg);
    updatePrevCtrl(ccs);

    double accel, steer;
    vehicle_cmd_msg_.header.frame_id = input_msg->header.frame_id;
    vehicle_cmd_msg_.header.stamp = input_msg->header.stamp;
    vehicle_cmd_msg_.header.seq++;
    vehicle_cmd_msg_.ctrl_cmd = ccs.cmd;
    vehicle_cmd_msg_.gear = 64;

    // convert measure from radian to normalized angle
    steer = - (ccs.cmd.steering_angle / PI * 180.0) / 39.4; 

    // limit accel
    accel = ccs.cmd.linear_acceleration / accel_divide_gain_;
    accel = std::max(deccel_limit_, std::min(accel, accel_limit_));
    // low pass filter
    filtered_accel_ = lowpass_gain_accl_ * filtered_accel_ + (1 - lowpass_gain_accl_) * accel;
    filtered_steer_ = lowpass_gain_steer_ * filtered_steer_ + (1 - lowpass_gain_steer_) * steer;
    vehicle_cmd_msg_.ctrl_cmd.linear_acceleration = filtered_accel_;
    vehicle_cmd_msg_.ctrl_cmd.steering_angle = filtered_steer_;
    
    // emergent brake
    if (brake_flag_) {
        vehicle_cmd_msg_.ctrl_cmd.linear_velocity = -10.0;
        vehicle_cmd_msg_.ctrl_cmd.linear_acceleration = -10.0;
    }
}


void LaneStopper::timer_callback(const ros::TimerEvent& e) {
    vehicle_cmd_pub.publish(vehicle_cmd_msg_);
}

// --------------------------------------------------------------------------
boost::optional<double> LaneStopper::calcLaccWithSteeringAngle(const double& lv, const double& sa) const {
  if (std::fabs(wheel_base_) < MIN_LENGTH) {
    return boost::none;
  }
  return lv * lv * std::tan(sa) / wheel_base_;
}

boost::optional<double> LaneStopper::calcLjerkWithSteeringAngle(const double& lv, const double& sa) const {
  if (std::fabs(sa_prev_.dt) < MIN_DURATION || std::fabs(wheel_base_) < MIN_LENGTH) {
    return boost::none;
  }
  return lv * lv *
    ((std::tan(sa) - std::tan(sa_prev_.val)) / sa_prev_.dt) / wheel_base_;
}

void LaneStopper::checkCtrl(const autoware_msgs::ControlCommandStamped& msg) {
  const double lv = msg.cmd.linear_velocity;
  const double sa = msg.cmd.steering_angle;
  const auto lacc = calcLaccWithSteeringAngle(lv, sa);
  const auto ljerk = calcLjerkWithSteeringAngle(lv, sa);
  if (lacc) {
    health_checker_.CHECK_MAX_VALUE("ctrl_lateral_accel_high",
      lacc.get(), lateral_accel_limit_, 3 * lateral_accel_limit_, DBL_MAX,
      "lateral_accel is too high in ctrl filtering");
  }
  if (ljerk) {
    health_checker_.CHECK_MAX_VALUE("ctrl_lateral_jerk_high",
      lacc.get(), lateral_jerk_limit_, 3 * lateral_jerk_limit_, DBL_MAX,
      "lateral_jerk is too high in ctrl filtering");
  }
}

void LaneStopper::updatePrevCtrl(const autoware_msgs::ControlCommandStamped& msg) {
  sa_prev_.time = msg.header.stamp;
  sa_prev_.val = msg.cmd.steering_angle;
}

autoware_msgs::ControlCommandStamped LaneStopper::lateralLimitCtrl(const autoware_msgs::ControlCommandStamped& msg) {
  static bool init = false;

  autoware_msgs::ControlCommandStamped ccs;
  ccs = msg;

  ros::Time t = msg.header.stamp;
  sa_prev_.dt = (t - sa_prev_.time).toSec();
  const double lv = msg.cmd.linear_velocity;
  double sa = msg.cmd.steering_angle;

  // skip first msg, check linear_velocity
  const bool is_stopping = (std::fabs(lv) < MIN_LINEAR_X);
  if (!init || is_stopping)
  {
    init = true;
    return ccs;
  }

  // lateral acceleration
  double lacc = calcLaccWithSteeringAngle(lv, sa).get();
  // limit lateral acceleration
  if (std::fabs(lacc) > lateral_accel_limit_ && !is_stopping)
  {
    double sgn = lacc / std::fabs(lacc);
    double sa_max =
      std::atan(sgn * lateral_accel_limit_ * wheel_base_ / (lv * lv));
    ROS_WARN_THROTTLE(1,
      "Limit steering angle by lateral acceleration: %f -> %f", sa, sa_max);
    sa = sa_max;
  }

  // lateral jerk
  double ljerk = calcLjerkWithSteeringAngle(lv, sa).get();
  // limit lateral jerk
  if (std::fabs(ljerk) > lateral_jerk_limit_ && !is_stopping)
  {
    double sgn = ljerk / std::fabs(ljerk);
    double sa_max = std::atan(std::tan(sa_prev_.val) +
      sgn * (lateral_jerk_limit_ * wheel_base_ / (lv * lv)) * sa_prev_.dt);
    ROS_WARN_THROTTLE(1,
      "Limit steering angle by lateral jerk: %f -> %f", sa, sa_max);
    sa = sa_max;
  }

  // update by lateral limitaion
  ccs.cmd.steering_angle = sa;

  // update lateral acceleration/jerk
  lacc = calcLaccWithSteeringAngle(lv, sa).get();
  ljerk = calcLjerkWithSteeringAngle(lv, sa).get();

  return ccs;
}

// END -------------------------------------------------------------------------- 


LaneStopper::LaneStopper() : node_handle_(), private_node_handle_("~"), tf_listener_(), brake_flag_(false), 
filtered_accel_(0.0), filtered_steer_(0.0), health_checker_(node_handle_, private_node_handle_) {
    private_node_handle_.param("stop_distance", stop_distance_, 20.0);
    private_node_handle_.param("objects_topic", objects_topic_, std::string("/detection/lidar_detector/objects"));
    private_node_handle_.param("current_pose_topic", pose_topic_, std::string("/current_pose"));
    private_node_handle_.param("accel_divide_gain", accel_divide_gain_, 20.0);
    private_node_handle_.param("lowpass_gain_accl", lowpass_gain_accl_, 0.0);
    private_node_handle_.param("accel_limit", accel_limit_, 1.0);
    private_node_handle_.param("deccel_limit", deccel_limit_, -1.0);
    private_node_handle_.param("lowpass_gain_steer", lowpass_gain_steer_, 0.0);

    node_handle_.param("vehicle_info/wheel_base", wheel_base_, 2.7);
    private_node_handle_.param("lateral_accel_limit", lateral_accel_limit_, 5.0);
    private_node_handle_.param("lateral_jerk_limit", lateral_jerk_limit_, 5.0);
    private_node_handle_.param("loop_rate", loop_rate_, 30.0);

    LaneStopper::reset_vehicle_cmd_msg();
    health_checker_.ENABLE();
}

void LaneStopper::run() {
    objectArraySubscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_, objects_topic_, 1);
    poseSubscriber_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_handle_, pose_topic_, 1);
    synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *objectArraySubscriber_, *poseSubscriber_);
    synchronizer_->registerCallback(boost::bind(&LaneStopper::syncedDetectionsCallback, this, _1, _2));

    vehicle_cmd_sub_ = node_handle_.subscribe("ctrl_raw", 1, &LaneStopper::CtrlCmdCallback, this);

    bool_publisher = node_handle_.advertise<std_msgs::Bool>("/brake_flag", 10); // to avoid miss subscription
    vehicle_cmd_pub = node_handle_.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 1, true);
    timer_ = node_handle_.createTimer(ros::Duration(1.0 / loop_rate_), &LaneStopper::timer_callback, this);

    ros::spin();
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "lane_stopper");
  LaneStopper node;
  node.run();
  return 0;
}