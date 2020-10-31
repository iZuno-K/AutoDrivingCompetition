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
#include <autoware_msgs/ControlCommandStamped.h> 
#include <autoware_msgs/VehicleCmd.h> // publish acclel command

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class VehicleOdmController {
public:
    VehicleOdmController();
    void run();

private:
  // Def synclonizers
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, autoware_msgs::VehicleCmd> SyncPolicyT;
  message_filters::Synchronizer<SyncPolicyT> *synchronizer_;
  message_filters::Subscriber<nav_msgs::Odometry> *sub_odom_;
  message_filters::Subscriber<autoware_msgs::VehicleCmd> *sub_vcmd_;

  // Not ROS specific func.
    void init_odom();
    void init_vcmd();

  // Callback and the method of which
    void odm_callback(const nav_msgs::Odometry::Ptr& input);
    void vehicle_cmd_callback(const autoware_msgs::VehicleCmd::Ptr& input);

    ros::NodeHandle nh;
    ros::Subscriber odm_sub, vehicle_cmd_sub;
    ros::Publisher f_vel_pub;

    ros::Time previous_time;
    double vx_odm, vx_odm_prev, ax_odm, ax_odm_prev;
    double vx_cmd, vx_cmd_prev, ax_cmd, ax_cmd_prev;
};

#endif //TEST_VEHICLE_ODM_CONTROLLER_H