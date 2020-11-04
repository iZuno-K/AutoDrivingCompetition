/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "points_densifier.h"
#define __APP_NAME__ "points_densifier"

using namespace std;

PointsDensifier::PointsDensifier() : node_handle_(), private_node_handle_("~"), tf_listener_(), index_(0), max_index_(0){
    private_node_handle_.param("fixed_frame_id", fixed_frame_id_, string("world"));
    private_node_handle_.param("output_frame_id", output_frame_id_, string("top_velodyne_link"));
    private_node_handle_.param("history_num", history_num_, 5);
    private_node_handle_.param("remove_backward_points", remove_backward_flag_, false);
    
    if (MAX_HISTORY_NUM < history_num_) {
        ROS_WARN("[%s] history_num_(%d) decreaased into MAX_HISTORY_NUM(%d)", __APP_NAME__, history_num_, MAX_HISTORY_NUM);
        history_num_ = MAX_HISTORY_NUM;
    }

    cloud_subscriber_ = node_handle_.subscribe<PointCloudMsgT>("/points_in", 1, &PointsDensifier::pointcloud_callback, this);
    cloud_publisher_ = node_handle_.advertise<PointCloudMsgT>("/points_densified", 1);
}

void PointsDensifier::pointcloud_callback(const PointCloudMsgT::ConstPtr &msg) {
    PointCloudT::Ptr cloud_densified(new PointCloudT);

    // transform points
    try {
        cloud_history[index_] = PointCloudT().makeShared();
        pcl::fromROSMsg(*msg, *cloud_history[index_]);
        // convert point cloud in fixed frame
        if (msg->header.frame_id != fixed_frame_id_) {
            tf_listener_.waitForTransform(fixed_frame_id_, msg->header.frame_id, ros::Time(0), ros::Duration(1.0));
            pcl_ros::transformPointCloud(fixed_frame_id_, ros::Time(0), *cloud_history[index_], msg->header.frame_id, *cloud_history[index_], tf_listener_);
        }
        cloud_history[index_]->header = pcl_conversions::toPCL(msg->header);
        cloud_history[index_]->header.frame_id = fixed_frame_id_;
        for (size_t i = 0; i < (size_t)min(history_num_, max_index_); ++i) {
            cloud_history[i]->header = cloud_history[index_]->header;
        }
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
        return;
    }

    // remove backward points
    if (remove_backward_flag_) {
        for (size_t i = 0; i < (size_t)min(history_num_, max_index_ + 1); ++i) {
            PointCloudT::Ptr cloud_converted(new PointCloudT);
            try {
                tf_listener_.waitForTransform("base_link", cloud_history[i]->header.frame_id, ros::Time(0), ros::Duration(1.0));
                pcl_ros::transformPointCloud("base_link", ros::Time(0), *cloud_history[i], cloud_history[i]->header.frame_id, *cloud_converted, tf_listener_);
                // auto result = std::remove_if(cloud_converted->points.begin())
                int count = 0;
                for (int j = 0; j < (int)cloud_converted->points.size(); j++) {
                // for (auto &pc : cloud_history) {
                    if (cloud_converted->points[j].x < 0 || (cloud_converted->points[j].x < 5 && cloud_converted->points[j].y < 2 && cloud_converted->points[j].y > -2)) {
                        cloud_history[i]->points.erase(cloud_history[i]->points.begin() + j - count);  // 消した分indexずれるので調整
                        count++;
                    }

                }
                // ROS_INFO("[%s] remove points from (%d) to (%d)", __APP_NAME__, (int)cloud_converted->points.size(), (int)cloud_history[i]->points.size());
                // extract.setInputCloud(cloud_history[i]);
                // extract.setIndices(inliers);
                // extract.setNegative(true);
                // extract.filter(*cloud_history[i);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
                return;
            }
        }
    }

    // merge points
    for (size_t i = 0; i < (size_t)min(history_num_, max_index_ + 1); ++i) {
        *cloud_densified += *cloud_history[i];
    }
    cloud_densified->header = pcl_conversions::toPCL(msg->header);
    cloud_densified->header.frame_id = fixed_frame_id_;
    try {
        tf_listener_.waitForTransform(output_frame_id_, fixed_frame_id_, ros::Time(0), ros::Duration(1.0));
        pcl_ros::transformPointCloud(output_frame_id_, ros::Time(0), *cloud_densified, fixed_frame_id_, *cloud_densified, tf_listener_);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("[%s: after merge] %s", __APP_NAME__, ex.what());
        return;
    }

    index_++;
    if (index_ >= history_num_) index_ = 0;
    if (max_index_ < history_num_) max_index_++;

    // publsh points
    cloud_densified->header = pcl_conversions::toPCL(msg->header);
    cloud_densified->header.frame_id = output_frame_id_;
    cloud_publisher_.publish(cloud_densified);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_densifier");
  PointsDensifier node;
  ros::spin();
  return 0;
}
