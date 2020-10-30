#ifndef TEST_VEHICLE_ODM_CONTROLLER_H
#define TEST_VEHICLE_ODM_CONTROLLER_H

// #include <sstream>

// ROS packages
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
    double calcDiffForRadian(const double lhs_rad, const double rhs_rad);
    double wrapToPm(double a_num, const double a_max);
    double wrapToPmPi(double a_angle_rad);
    void init_varibles();

  // Callback and the method of which
    void odm_callback(const sensor_msgs::Imu::Ptr& input);
    void calc_vel_from_imu(ros::Time current_time);
    void set_twist_stamped();
    void vel_pub_timer(const ros::TimerEvent& e);

    ros::NodeHandle nh;
    ros::Subscriber imu_sub_;
    ros::Publisher f_vel_pub;
    // ros::NodeHandle private_nh("~");

    nav_msgs::Odometry odom;
    ros::Time previous_time;
    double current_velocity_imu_x;
    double current_velocity_imu_y;
    double current_velocity_imu_z;
    double current_angular_velocity_imu_x;
    double current_angular_velocity_imu_y;
    double current_angular_velocity_imu_z;
    double previous_imu_roll;
    double previous_imu_pitch;
    double previous_imu_yaw;
    double offset_imu_x, offset_imu_y, offset_imu_z, offset_imu_roll, offset_imu_pitch, offset_imu_yaw;
};

#endif //TEST_VEHICLE_ODM_CONTROLLER_H