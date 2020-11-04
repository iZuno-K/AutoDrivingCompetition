#include <cmath>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include "std_msgs/Bool.h"
#include <autoware_msgs/ControlCommandStamped.h>
#include <autoware_msgs/VehicleCmd.h>
#include <boost/optional.hpp>
#include <autoware_health_checker/health_checker/health_checker.h>
#include <time.h>

#define PI 3.14159265359

// const values
constexpr double MIN_LINEAR_X = 1e-3;
constexpr double MIN_LENGTH = 1e-3;
constexpr double MIN_DURATION = 1e-3;

struct StampedValue
{
  ros::Time time;
  double dt;
  double val;
  StampedValue() : time(0.0), dt(0.0), val(0.0) {}
  void reset()
  {
    time = ros::Time(0.0);
    val = 0.0;
  }
};

class LaneStopper {
    typedef message_filters::sync_policies::ApproximateTime<autoware_msgs::DetectedObjectArray,
                                                            geometry_msgs::PoseStamped> SyncPolicyT;

    ros::NodeHandle node_handle_, private_node_handle_;
    autoware_health_checker::HealthChecker health_checker_;

    message_filters::Subscriber<autoware_msgs::DetectedObjectArray> *objectArraySubscriber_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> *poseSubscriber_;
    message_filters::Synchronizer<SyncPolicyT> *synchronizer_;
    ros::Subscriber ctrl_cmd_sub_, velocity_sub_, pose_sub_;
    ros::Timer timer_;
    ros::Time previous_time;

    ros::Publisher bool_publisher;
    ros::Publisher vehicle_cmd_pub;

    tf::TransformListener tf_listener_;
    
    bool poseDontCareLane(const geometry_msgs::PoseStamped& msg);
    bool objectInIncomingLane(const autoware_msgs::DetectedObject &object);
    bool poseInIncomingLane(const geometry_msgs::PoseStamped& msg);
    bool objectIncomingIntersection2(const autoware_msgs::DetectedObject &object);
    bool objectIncomingIntersection1(const autoware_msgs::DetectedObject &object);
    bool inIntersection1(const geometry_msgs::PoseStamped& msg);
    bool inIntersection2(const geometry_msgs::PoseStamped& msg);
    double dist(const geometry_msgs::Pose p, const geometry_msgs::Pose q);
    bool stopIntersection(const autoware_msgs::DetectedObjectArray &input, const geometry_msgs::PoseStamped& mycarpose);
    void syncedDetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &input,
                             const geometry_msgs::PoseStamped::ConstPtr& mycarpose);
    void CtrlCmdCallback(const autoware_msgs::ControlCommandStampedConstPtr& input_msg);
    void modify_vehicle_cmd();
    void publish_vehicle_cmd();
    void callbackFromOdom(const nav_msgs::Odometry::ConstPtr& msg);
    void callbackFromPose(const geometry_msgs::PoseStamped::ConstPtr& mycarpose);
    void timer_callback(const ros::TimerEvent& e);
    boost::optional<double> calcLaccWithSteeringAngle(const double& lv, const double& sa) const;
    boost::optional<double> calcLjerkWithSteeringAngle(const double& lv, const double& sa) const;
    void checkCtrl(const autoware_msgs::ControlCommandStamped& msg);
    void updatePrevCtrl(const autoware_msgs::ControlCommandStamped& msg);
    autoware_msgs::ControlCommandStamped lateralLimitCtrl(const autoware_msgs::ControlCommandStamped& msg);

    void reset_vehicle_cmd_msg();
    const double a[6] = {-0.23597189174169617,
                        -0.26430773408498043,
                         -0.3739331552065404,
                        4.453371279902778,
                        3.7920438197948,
                        3.999999998458709,
                        };
    const double b[6] = {-361.04047744482205,
                        -355.9439251504078,
                        -360.16951273140984,
                        -112.07512446590951,
                        -212.38461245157148,
                        101.0462950010089
                        };

    double stop_distance_;
    std::string objects_topic_;
    std::string pose_topic_;
    bool brake_flag_;
    double accel_divide_gain_;
    double filtered_accel_, filtered_steer_;
    double lowpass_gain_accl_, lowpass_gain_steer_;
    double accel_limit_, deccel_limit_;
    double loop_rate_, lateral_accel_limit_, lateral_jerk_limit_, wheel_base_;
    double initial_wait_time_;
    bool flag_activate_;
    bool is_in_intersection1_, is_in_intersection2_, is_me_in_incomming_lane_, dont_care_lane_;
    float intersection_force_accel_;
    time_t start_time_;
    double current_linear_velocity_;
    bool is_velocity_set_;
    // dataset
    StampedValue sa_prev_;

    autoware_msgs::VehicleCmd vehicle_cmd_msg_;
    public:
        void run();
        LaneStopper();
};
