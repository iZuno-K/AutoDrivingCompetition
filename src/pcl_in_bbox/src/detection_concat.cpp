#include "detection_concat/detection_concat.h"

DetectionConcatinator::DetectionConcatinator() : nh(), private_nh("~") {
    private_nh.param<std::string>("topic1", topic1_, "/detection/lidar_detector/objects");
    private_nh.param<std::string>("topic2", topic2_, "/detection/lidar_detector/objects2");
    publish_concat_ = nh.advertise<autoware_msgs::DetectedObjectArray>("/detection/concat", 1);

    detect_subscriber1_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(nh, topic1_, 1);
    detect_subscriber2_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(nh, topic2_, 1);
    synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *detect_subscriber1_, *detect_subscriber2_);
	synchronizer_->registerCallback(boost::bind(&DetectionConcatinator::callback, this, _1, _2));
}

void DetectionConcatinator::callback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_1,
                                     const autoware_msgs::DetectedObjectArray::ConstPtr &in_2) {
    autoware_msgs::DetectedObjectArray detections;
    if (in_1->header.frame_id != in_2->header.frame_id) {
        ROS_ERROR("[%s] Assume both of frame_ids are the same, but different", __APP_NAME__);
    }
    else {
        detections.header = in_1->header;
        for (size_t i = 0; i < in_1->objects.size(); i++) detections.objects.push_back(in_1->objects[i]);
        for (size_t i = 0; i < in_2->objects.size(); i++) detections.objects.push_back(in_2->objects[i]);
        publish_concat_.publish(detections);
    }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "detection_concat");
  DetectionConcatinator node;
  ros::spin();
  return 0;
}
