#include "vehicle_state.hpp"
#define __APP_NAME__ "vehicle_odm_controller"

VehicleOdmController::VehicleOdmController(){
  init_varibles();
}

void VehicleOdmController::init_varibles(){
  previous_time =  ros::Time::now();
  vx = 0;
  vth = 0;
  imu_sub_ = nh.subscribe("odom", 10, &VehicleOdmController::odm_callback, this);
  // f_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("fast_current_vel", 10);
}

void VehicleOdmController::odm_callback(const nav_msgs::Odometry::Ptr& input)
{
  // std::cout << __func__ << std::endl;
  const ros::Time current_time = input->header.stamp;
  const double diff_time = (current_time - previous_time).toSec();

  // read velocity
  vx =  input -> twist.twist.linear.x;
  std::cout << vx << "\n";
  previous_time = current_time;
}

void VehicleOdmController::run() {
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    std::cout << vx << "\n";
    ros::spinOnce();
    loop_rate.sleep();
  }
}
