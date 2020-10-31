#include "vehicle_state.hpp"
#define __APP_NAME__ "vehicle_odm_controller"

VehicleOdmController::VehicleOdmController(){
  init_odom();
  // init_vcmd();
  loop_rate_ = 12.0;
  vcmd_pub = nh.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 1, true);
  timer_ = nh.createTimer(ros::Duration(1.0 / loop_rate_), &VehicleOdmController::timer_callback, this);

  init_sync();
}

void VehicleOdmController::init_odom(){
  time_odom_prev = ros::Time::now();
  vx_odm = 0;
  vx_odm_prev = 0;
  ax_odm = 0;
  ax_odm_prev = 0;
  odm_sub = nh.subscribe("odom", 10, &VehicleOdmController::odm_callback, this);
}

void VehicleOdmController::init_vcmd(){
  vx_cmd = 0;
  ax_cmd = 0;
  vehicle_cmd_sub = nh.subscribe("vehicle_cmd", 10, &VehicleOdmController::vehicle_cmd_callback, this);
}

void VehicleOdmController::odm_callback(const nav_msgs::Odometry::Ptr& input)
{
  const ros::Time current_time = input->header.stamp;
  vx_odm =  input -> twist.twist.linear.x;
  const double diff_time = (current_time - time_odom_prev).toSec();
  if (diff_time != 0){
    ax_odm = (vx_odm - vx_odm_prev)/ diff_time;
  } else {
    ax_odm = ax_odm_prev;
  }
  time_odom_prev = current_time;
  vx_odm_prev = vx_odm;
  ax_odm_prev = ax_odm;
  vehicle_cmd_msg_.header.stamp = input->header.stamp;
}

void VehicleOdmController::vehicle_cmd_callback(const autoware_msgs::VehicleCmd::Ptr& input){
  // const ros::Time current_time = input->header.stamp;
  vx_cmd = input->ctrl_cmd.linear_velocity;
  ax_cmd = input->ctrl_cmd.linear_acceleration;
}

void VehicleOdmController::timer_callback(const ros::TimerEvent& e) {
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
  vehicle_cmd_msg_.ctrl_cmd.linear_acceleration = 0.1;
  vehicle_cmd_msg_.ctrl_cmd.steering_angle = 0.0;
  vehicle_cmd_msg_.emergency = 0.0;
  vcmd_pub.publish(vehicle_cmd_msg_);
}


void VehicleOdmController::init_sync(){
  time_odom_prev = ros::Time::now();
  vx_odm = 0;
  vx_odm_prev = 0;
  ax_odm = 0;
  ax_odm_prev = 0;
  vx_cmd = 0;
  ax_cmd = 0;
  sub_odom_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/odom", 1);
  sub_vcmd_ = new message_filters::Subscriber<autoware_msgs::VehicleCmd>(nh, "/vehicle_cmd", 1);
  synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *sub_odom_, *sub_vcmd_);
	synchronizer_->registerCallback(boost::bind(&VehicleOdmController::sync_odm_cmd_callback, this, _1, _2));
};


void VehicleOdmController::sync_odm_cmd_callback(const nav_msgs::Odometry::ConstPtr &odom_ptr, const autoware_msgs::VehicleCmd::ConstPtr &cmd_ptr){
  // update odom
  ROS_ERROR("[%s] hogehoge", __APP_NAME__);
  std::cout << "OI\n";
  const ros::Time time_odom = odom_ptr->header.stamp;
  vx_odm =  odom_ptr -> twist.twist.linear.x;
  const double diff_time = (time_odom - time_odom_prev).toSec();
  if (diff_time != 0){
    ax_odm = (vx_odm - vx_odm_prev)/ diff_time;
  } else {
    ax_odm = ax_odm_prev;
  }
  time_odom_prev = time_odom;
  vx_odm_prev = vx_odm;
  ax_odm_prev = ax_odm;
  // update vcmd
  vx_cmd = cmd_ptr->ctrl_cmd.linear_velocity;
  ax_cmd = cmd_ptr->ctrl_cmd.linear_acceleration;
}

void VehicleOdmController::run() {
  ros::Rate loop_rate(10);
  ros::spin();
  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   // std::cout << vx_odm << "\t" << ax_odm << "\t";
  //   // std::cout << vx_cmd << "\t" << ax_cmd << "\n";
  //   loop_rate.sleep();
  // }
}
