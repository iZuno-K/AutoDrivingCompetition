
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <string>

#define __APP_NAME__ "detection_concat"

class DetectionConcatinator {
    ros::NodeHandle nh, private_nh;
    typedef message_filters::sync_policies::ApproximateTime<autoware_msgs::DetectedObjectArray,autoware_msgs::DetectedObjectArray> SyncPolicyT;
	message_filters::Subscriber<autoware_msgs::DetectedObjectArray> *detect_subscriber1_;
	message_filters::Subscriber<autoware_msgs::DetectedObjectArray> *detect_subscriber2_;
	message_filters::Synchronizer<SyncPolicyT> *synchronizer_;
    ros::Publisher publish_concat_;
    std::string topic1_, topic2_;

    void callback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_1,
                  const autoware_msgs::DetectedObjectArray::ConstPtr &in_2);
    public:
        DetectionConcatinator();
};