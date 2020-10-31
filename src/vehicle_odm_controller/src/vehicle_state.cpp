#include "vehicle_state.hpp"
#define __APP_NAME__ "vehicle_odm_controller"

VehicleOdmController::VehicleOdmController(){
  init_odom();
}

void VehicleOdmController::init_odom(){
  previous_time =  ros::Time::now();
  vx = 0;
  pre_vx = 0;
  vth = 0;
  ax = 0;
  pre_ax = 0;
  odm_sub = nh.subscribe("odom", 10, &VehicleOdmController::odm_callback, this);
  vehicle_cmd_sub = nh.subscribe("vehicle_cmd", 10, &VehicleOdmController::vehicle_cmd_callback, this);
}

void VehicleOdmController::odm_callback(const nav_msgs::Odometry::Ptr& input)
{
  const ros::Time current_time = input->header.stamp;
  vx =  input -> twist.twist.linear.x;

  const double diff_time = (current_time - previous_time).toSec();
  if (diff_time != 0){
    ax = (vx - pre_vx)/ diff_time;
  } else {
    ax = pre_ax;
  }
  previous_time = current_time;
  pre_vx = vx;
  pre_ax = ax;
}

void VehicleOdmController::vehicle_cmd_callback(const autoware_msgs::VehicleCmd::Ptr& input){

}

void VehicleOdmController::run() {
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std::cout << vx << "\t" << ax << "\n";
    ros::spinOnce();
    loop_rate.sleep();
  }
}
