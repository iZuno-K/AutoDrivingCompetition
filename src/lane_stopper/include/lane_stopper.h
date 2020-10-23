#include <cmath>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

#define STOP_DISTSTANCE 15

class LaneStopper {
    typedef message_filters::sync_policies::ApproximateTime<autoware_msgs::DetectedObjectArray,
                                                            geometry_msgs::PoseStamped> SyncPolicyT;

    ros::NodeHandle node_handle_, private_node_handle_;
    message_filters::Subscriber<autoware_msgs::DetectedObjectArray> *objectArraySubscriber_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> *poseSubscriber_;
    message_filters::Synchronizer<SyncPolicyT> *synchronizer_;

    ros::Publisher debug_publisher1;
    ros::Publisher debug_publisher2;

    tf::TransformListener tf_listener_;
    
    bool objectInIncomingLane(const autoware_msgs::DetectedObject &object);
    bool objectIncomingIntersection2(const autoware_msgs::DetectedObject &object);
    bool inIntersection1(const geometry_msgs::PoseStamped& msg);
    bool inIntersection2(const geometry_msgs::PoseStamped& msg);
    double dist(const geometry_msgs::Pose p, const geometry_msgs::Pose q);
    bool stopIntersection1(const autoware_msgs::DetectedObjectArray &input, const geometry_msgs::PoseStamped& mycarpose);
    void SyncedDetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &input,
                             const geometry_msgs::PoseStamped::ConstPtr& mycarpose);
    const double a[5] = {-0.2582730920132578,
                        -0.26430773408498043,
                        -0.24750019525741684,
                        3.9263219260739675,
                        3.7920438197948,
                        };
    const double b[5] = {-361.2845319741398,
                        -355.9439251504078,
                        -347.22055328879355,
                        -160.03610323652867,
                        -212.38461245157148
                        };
    public:
        void run();
        LaneStopper();
};
