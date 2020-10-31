#include "vehicle_state.hpp"
#define __APP_NAME__ "vehicle_odm_controller"

VehicleOdmController::VehicleOdmController(){
  init_odom();
}

void VehicleOdmController::init_odom(){
  previous_time =  ros::Time::now();
  vx_odm = 0;
  vx_odm_prev = 0;
  ax_odm = 0;
  ax_odm_prev = 0;
  odm_sub = nh.subscribe("odom", 10, &VehicleOdmController::odm_callback, this);
}

void VehicleOdmController::odm_callback(const nav_msgs::Odometry::Ptr& input)
{
  const ros::Time current_time = input->header.stamp;
  vx_odm =  input -> twist.twist.linear.x;
  const double diff_time = (current_time - previous_time).toSec();
  if (diff_time != 0){
    ax_odm = (vx_odm - vx_odm_prev)/ diff_time;
  } else {
    ax_odm = ax_odm_prev;
  }
  previous_time = current_time;
  vx_odm_prev = vx_odm;
  ax_odm_prev = ax_odm;
}

void VehicleOdmController::vehicle_cmd_callback(const autoware_msgs::VehicleCmd::Ptr& input){
  // const ros::Time current_time = input->header.stamp;
  vx_cmd = input->ctrl_cmd.linear_velocity;
  ax_cmd = input->ctrl_cmd.linear_acceleration;
}

void VehicleOdmController::run() {
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std::cout << vx_odm << "\t" << ax_odm << "\t";
    std::cout << vx_cmd << "\t" << ax_cmd << "\n";
    ros::spinOnce();
    loop_rate.sleep();
  }
}


// #include <nav_msgs/Odometry.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <autoware_msgs/DetectedObjectArray.h>
// #include <autoware_msgs/VehicleCmd.h>
// typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, autoware_msgs::VehicleCmd> SyncPolicyT;
// message_filters::Subscriber<nav_msgs::Odometry> *sub_odom_;
// message_filters::Subscriber<autoware_msgs::VehicleCmd> *sub_vcmd_;
// message_filters::Synchronizer<SyncPolicyT> *synchronizer_;
// void callback(const nav_msgs::Odometry::ConstPtr &in_1,
//                   const autoware_msgs::DetectedObjectArray::ConstPtr &in_2);
// void init() {
//     sub_odom_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(nh, topic1_, 1);
//     sub_vcmd_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(nh, topic2_, 1);
//     synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *sub_odom_, *sub_vcmd_);
// 	synchronizer_->registerCallback(boost::bind(&CLASS::callback, this, _1, _2));
// }