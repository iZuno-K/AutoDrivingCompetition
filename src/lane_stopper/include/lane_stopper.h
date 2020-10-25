#include <cmath>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include "std_msgs/Bool.h"
#include "autoware_msgs/VehicleCmd.h"

#define PI 3.14159265359


class LaneStopper {
    typedef message_filters::sync_policies::ApproximateTime<autoware_msgs::DetectedObjectArray,
                                                            geometry_msgs::PoseStamped> SyncPolicyT;

    ros::NodeHandle node_handle_, private_node_handle_;
    message_filters::Subscriber<autoware_msgs::DetectedObjectArray> *objectArraySubscriber_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> *poseSubscriber_;
    message_filters::Synchronizer<SyncPolicyT> *synchronizer_;
    ros::Subscriber vehicle_cmd_sub_;

    ros::Publisher bool_publisher;
    ros::Publisher vehicle_cmd_pub;

    tf::TransformListener tf_listener_;
    
    bool objectInIncomingLane(const autoware_msgs::DetectedObject &object);
    bool objectIncomingIntersection2(const autoware_msgs::DetectedObject &object);
    bool objectIncomingIntersection1(const autoware_msgs::DetectedObject &object);
    bool inIntersection1(const geometry_msgs::PoseStamped& msg);
    bool inIntersection2(const geometry_msgs::PoseStamped& msg);
    double dist(const geometry_msgs::Pose p, const geometry_msgs::Pose q);
    bool stopIntersection(const autoware_msgs::DetectedObjectArray &input, const geometry_msgs::PoseStamped& mycarpose);
    void syncedDetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &input,
                             const geometry_msgs::PoseStamped::ConstPtr& mycarpose);
    void vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& input_msg);
    void reset_vehicle_cmd_msg();
    const double a[6] = {-0.2582730920132578,
                        -0.26430773408498043,
                        -0.24750019525741684,
                        4.453371279902778,
                        3.7920438197948,
                        3.999999998458709,
                        };
    const double b[6] = {-361.2845319741398,
                        -355.9439251504078,
                        -347.22055328879355,
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
    double lowpass_gain_, lowpass_gain_steer_;
    double accel_limit_, deccel_limit_;

    autoware_msgs::VehicleCmd twist_gate_msg_;
    public:
        void run();
        LaneStopper();
};
