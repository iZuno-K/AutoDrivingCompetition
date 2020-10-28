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

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <velodyne_pointcloud/point_types.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <vector>

using namespace std;

#define MAX_HISTORY_NUM 10

class PointsDensifier
{
public:
  PointsDensifier();

private:
  typedef pcl::PointXYZI PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef sensor_msgs::PointCloud2 PointCloudMsgT;

  ros::NodeHandle node_handle_, private_node_handle_;
  ros::Subscriber cloud_subscriber_; 
  ros::Publisher cloud_publisher_;
  tf::TransformListener tf_listener_;

  std::string fixed_frame_id_;
  std::string output_frame_id_;

  PointCloudT::Ptr cloud_history[MAX_HISTORY_NUM];
  int index_;
  int max_index_;
  int history_num_;

  void pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1);
};
