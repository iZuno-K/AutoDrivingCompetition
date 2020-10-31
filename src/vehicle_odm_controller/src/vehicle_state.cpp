#include "vehicle_state.hpp"
#define __APP_NAME__ "vehicle_odm_controller"

VehicleOdmController::VehicleOdmController(){
  init_varibles();
}

void VehicleOdmController::init_varibles(){
  previous_time =  ros::Time::now();
  vx = 0;
  pre_vx = 0;
  vth = 0;
  ax = 0;
  pre_ax = 0;
  imu_sub_ = nh.subscribe("odom", 10, &VehicleOdmController::odm_callback, this);
  // f_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("fast_current_vel", 10);
}

void VehicleOdmController::odm_callback(const nav_msgs::Odometry::Ptr& input)
{
  // std::cout << __func__ << std::endl;
  // read out
  const ros::Time current_time = input->header.stamp;
  vx =  input -> twist.twist.linear.x;

  const double diff_time = (current_time - previous_time).toSec();
  if (diff_time != 0){
    ax = (vx - pre_vx)/ diff_time;
  } else {
    ax = pre_ax;
  }

  // std::cout << vx <<  ", " << ax << ", " << "\n";
  previous_time = current_time;
  pre_vx = vx;
  pre_ax = ax;
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
