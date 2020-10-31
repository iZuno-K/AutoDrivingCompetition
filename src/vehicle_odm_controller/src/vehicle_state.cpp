#include "vehicle_state.hpp"
#define __APP_NAME__ "vehicle_odm_controller"

VehicleOdmController::VehicleOdmController(){
  // init_odom();
  // init_vcmd();
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
}

void VehicleOdmController::vehicle_cmd_callback(const autoware_msgs::VehicleCmd::Ptr& input){
  // const ros::Time current_time = input->header.stamp;
  vx_cmd = input->ctrl_cmd.linear_velocity;
  ax_cmd = input->ctrl_cmd.linear_acceleration;
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
  while (ros::ok())
  {
    ros::spinOnce();
    std::cout << vx_odm << "\t" << ax_odm << "\t";
    std::cout << vx_cmd << "\t" << ax_cmd << "\n";
    loop_rate.sleep();
  }
}
