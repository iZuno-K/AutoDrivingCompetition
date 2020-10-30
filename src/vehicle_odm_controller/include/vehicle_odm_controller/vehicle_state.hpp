#ifndef TEST_VEHICLE_ODM_CONTROLLER_H
#define TEST_VEHICLE_ODM_CONTROLLER_H

// #include <sstream>

// ROS packages
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h> // lsiten odometory
#include <autoware_msgs/ControlCommandStamped.h> // publish acclel command

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class VehicleOdmController {
public:
    VehicleOdmController();
    void run();

private:
  // Not ROS specific func.
    void init_varibles();

  // Callback and the method of which
    void odm_callback(const nav_msgs::Odometry::Ptr& input);

    ros::NodeHandle nh;
    ros::Subscriber imu_sub_;
    ros::Publisher f_vel_pub;
    // ros::NodeHandle private_nh("~");

    nav_msgs::Odometry odom;
    ros::Time previous_time;
    double vx, vth;

};

#endif //TEST_VEHICLE_ODM_CONTROLLER_H