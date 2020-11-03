#include "lane_stopper.h"
#define __APP_NAME__ "lane_stopper"


// 障害物検知しないとbrake flagが更新されないので、その対策 
bool LaneStopper::poseDontCareLane(const geometry_msgs::PoseStamped& msg) {
    geometry_msgs::Point p = msg.pose.position;
    if (a[1] * p.x + b[1] < p.y) return true;
    else return false;
}

bool LaneStopper::objectInIncomingLane(const autoware_msgs::DetectedObject &object) {
    double x, y;
    x = object.pose.position.x;
    y = object.pose.position.y; 
    if ((a[0] * x + b[0] < y) && (y < a[1] * x + b[1])) return true;
    else return false;
}

bool LaneStopper::poseInIncomingLane(const geometry_msgs::PoseStamped& msg) {
    geometry_msgs::Point p = msg.pose.position;
    if ((a[0] * p.x + b[0] < p.y) && (p.y < a[1] * p.x + b[1])) return true;
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


// STASRT copy from twist gate ========================================================================
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

// END ========================================================================

bool LaneStopper::stopIntersection(const autoware_msgs::DetectedObjectArray &input, const geometry_msgs::PoseStamped& mycarpose) {
  if (!dont_care_lane_) {
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
                  is_in_intersection1_ = inIntersection1(mycarpose);
                  if (is_in_intersection1_) {
                      if (objectIncomingIntersection1(obj)) {
                          if (dist(obj.pose, mycarpose.pose) < stop_distance_) {
                              // make_stop_waypoints();
                              ROS_ERROR("[%s] Stop at Intersection1.", __APP_NAME__);
                              return true;
                          }
                      }
                  }
                  // 自車が交差点2にいるとき
                  is_in_intersection2_ = inIntersection2(mycarpose);
                  if (is_in_intersection2_) {
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

void LaneStopper::callbackFromOdom(const nav_msgs::Odometry::ConstPtr& msg) {
  current_linear_velocity_ = msg->twist.twist.linear.x;
  is_velocity_set_ = true;
}

void LaneStopper::callbackFromPose(const geometry_msgs::PoseStamped::ConstPtr& mycarpose) {
  is_in_intersection1_ = inIntersection1(*mycarpose);
  is_in_intersection2_ = inIntersection2(*mycarpose);
  is_me_in_incomming_lane_ = poseInIncomingLane(*mycarpose);
  dont_care_lane_ = poseDontCareLane(*mycarpose);
  if (dont_care_lane_) brake_flag_ = true;  // 交差点抜けたら強制的にbrake_flag解除
}

void LaneStopper::reset_vehicle_cmd_msg() {
  vehicle_cmd_msg_.twist_cmd.twist.linear.x = 0.0;
  vehicle_cmd_msg_.twist_cmd.twist.angular.z = 0.0;
  vehicle_cmd_msg_.mode = 0.0;
  vehicle_cmd_msg_.gear = 0.0;
  vehicle_cmd_msg_.lamp_cmd.l = 0.0;
  vehicle_cmd_msg_.lamp_cmd.r = 0.0;
  vehicle_cmd_msg_.accel_cmd.accel = 0.0;
  vehicle_cmd_msg_.brake_cmd.brake = 0.0;
  vehicle_cmd_msg_.steer_cmd.steer = 0.0;
  vehicle_cmd_msg_.ctrl_cmd.linear_velocity = 0.0;
  vehicle_cmd_msg_.ctrl_cmd.steering_angle = 0.0;
  vehicle_cmd_msg_.emergency = 0.0;
}


void LaneStopper::modify_vehicle_cmd() {
  // limit accel
  double accel, steer;
  ros::Duration duration= ros::Time::now() - previous_time;
  accel = vehicle_cmd_msg_.ctrl_cmd.linear_acceleration;
  steer = vehicle_cmd_msg_.ctrl_cmd.steering_angle;
  
  accel = std::max(deccel_limit_, std::min(accel, accel_limit_));
  // low pass filter
  filtered_accel_ = lowpass_gain_accl_ * filtered_accel_ + (1 - lowpass_gain_accl_) * accel;
  filtered_steer_ = lowpass_gain_steer_ * filtered_steer_ + (1 - lowpass_gain_steer_) * steer;
  vehicle_cmd_msg_.ctrl_cmd.linear_acceleration = filtered_accel_;
  vehicle_cmd_msg_.ctrl_cmd.steering_angle = filtered_steer_;

  // ROS_INFO("[%s]: target velocity   %lf,   current velocity   %lf", __APP_NAME__, vehicle_cmd_msg_.ctrl_cmd.linear_velocity, current_linear_velocity_);
  // accel GAINの調整
  double thresh_vel = 1.0;  //m/s max 8.3?? 
  if (is_velocity_set_) {
    // If the velocity at the next call will exceed 30 m/s, 
    // or the diff between target vel. and current vel. is less than 'thresh_vel', and
    // accel > 0, 
    // send a command to vehicle_cmd_msg_ 'set an acecel 0.'
    
    // if (current_linear_velocity_ * duration.toSec() + 0.5 * filtered_accel_ * duration.toSec() * duration.toSec() > vehicle_cmd_msg_.ctrl_cmd.linear_velocity &&
    //     vehicle_cmd_msg_.ctrl_cmd.linear_velocity - current_linear_velocity_ < thresh_vel && 
    //     accel > 0) {
    if (current_linear_velocity_ + filtered_accel_ * duration.toSec() > vehicle_cmd_msg_.ctrl_cmd.linear_velocity - thresh_vel &&
        accel > 0) {
      // vehicle_cmd_msg_.ctrl_cmd.linear_acceleration /= accel_divide_gain_;
      vehicle_cmd_msg_.ctrl_cmd.linear_acceleration = 0.0125;
    }
  }

  // START 交差点時のcmd処理 ===================================================================
  // emergent brake
  if (brake_flag_) {
    vehicle_cmd_msg_.ctrl_cmd.linear_velocity = -10.0;
    vehicle_cmd_msg_.ctrl_cmd.linear_acceleration = -10.0;
  }
  else {
    // 交差点に侵入ずみで、brake_flag立ってなければさっさと渡りきる
    if (is_in_intersection1_ || is_in_intersection2_ || is_me_in_incomming_lane_) {
      // force to across the interseciton as soon as possible, if there is no obstacle
      double thresh_max_vel = 20.0 / 3.6;  //m/s
      if (is_velocity_set_ && current_linear_velocity_ < thresh_max_vel) {
        vehicle_cmd_msg_.ctrl_cmd.linear_acceleration = intersection_force_accel_;
      }
    }
  }
  // END 交差点時のcmd処理 ===================================================================

  // update time.
  previous_time = ros::Time::now();
}


void LaneStopper::CtrlCmdCallback(const autoware_msgs::ControlCommandStampedConstPtr& input_msg) {
  health_checker_.NODE_ACTIVATE();
  autoware_msgs::ControlCommandStamped ccs;
  checkCtrl(*input_msg);
  ccs = lateralLimitCtrl(*input_msg);
  updatePrevCtrl(ccs);

  vehicle_cmd_msg_.header.frame_id = input_msg->header.frame_id;
  vehicle_cmd_msg_.header.stamp = input_msg->header.stamp;
  vehicle_cmd_msg_.ctrl_cmd = ccs.cmd;
  vehicle_cmd_msg_.gear = 64;

  // convert measure from radian to normalized angle
  vehicle_cmd_msg_.ctrl_cmd.steering_angle = - (ccs.cmd.steering_angle / PI * 180.0) / 39.4; 
  
  // publish_vehicle_cmd();
}

void LaneStopper::timer_callback(const ros::TimerEvent& e) {
  publish_vehicle_cmd();
}

void LaneStopper::publish_vehicle_cmd() {
  modify_vehicle_cmd();
  // 最初起動何秒間かはpublish行わずにnode展開仕切るのをまつ
  if (!flag_activate_) {
    // ROS_WARN("[%s]: duration count   %ld", __APP_NAME__, (time(NULL) - start_time_));
    // ROS_WARN("[%s]: duration seconds %lf", __APP_NAME__, (double)(time(NULL) - start_time_) / CLOCKS_PER_SEC);
    if ((double)(time(NULL) - start_time_) > initial_wait_time_) {
        flag_activate_ = true;
    }
  }
  if (flag_activate_) {
    vehicle_cmd_pub.publish(vehicle_cmd_msg_);
  }
}

LaneStopper::LaneStopper() : node_handle_(), private_node_handle_("~"), tf_listener_(), brake_flag_(false), 
filtered_accel_(0.0), filtered_steer_(0.0), health_checker_(node_handle_, private_node_handle_), flag_activate_(false),
is_in_intersection1_(false), is_in_intersection2_(false), is_me_in_incomming_lane_(false), dont_care_lane_(true),
current_linear_velocity_(0.0), is_velocity_set_(false) {
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

    private_node_handle_.param("initial_wait_time", initial_wait_time_, 50.0);
    private_node_handle_.param("intersection_force_accel", intersection_force_accel_, 0.4f);
    start_time_ = time(NULL);
    previous_time = ros::Time::now();

    LaneStopper::reset_vehicle_cmd_msg();
    health_checker_.ENABLE();
}

void LaneStopper::run() {
    objectArraySubscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_, objects_topic_, 1);
    poseSubscriber_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_handle_, pose_topic_, 1);
    synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *objectArraySubscriber_, *poseSubscriber_);
    synchronizer_->registerCallback(boost::bind(&LaneStopper::syncedDetectionsCallback, this, _1, _2));

    ctrl_cmd_sub_ = node_handle_.subscribe("ctrl_raw", 1, &LaneStopper::CtrlCmdCallback, this);
    velocity_sub_ = node_handle_.subscribe("odom", 1, &LaneStopper::callbackFromOdom, this);
    pose_sub_ = node_handle_.subscribe("curren_pose", 1, &LaneStopper::callbackFromPose, this);

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
