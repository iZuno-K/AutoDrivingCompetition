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
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * pcl_in_bbox.cpp
 *
 *  Created on: May, 19th, 2018
 */


#include "pcl_in_bbox/pcl_in_bbox.h"

pcl::PointXYZ
ROSPixelCloudFusionApp::TransformPoint(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform)
{
	tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
	tf::Vector3 tf_point_t = in_transform * tf_point;
	return pcl::PointXYZ(tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
}


void ROSPixelCloudFusionApp::SyncedCallback(const sensor_msgs::PointCloud2::ConstPtr &in_cloud_msg,
                                            const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections) 
{
	image_frame_id_ = in_vision_detections->header.frame_id;

	if (image_frame_id_ == "")
	{
		ROS_INFO("[%s] Waiting for Image frame to be available.", __APP_NAME__);
		return;
	}
	if (!camera_lidar_tf_ok_)
	{
		camera_lidar_tf_ = FindTransform(image_frame_id_,
		                                 in_cloud_msg->header.frame_id);
	}
	if (!camera_info_ok_ || !camera_lidar_tf_ok_)
	{
		ROS_INFO("[%s] Waiting for Camera-Lidar TF and Intrinsics to be available.", __APP_NAME__);
		return;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*in_cloud_msg, *in_cloud);
	std::unordered_map<cv::Point, pcl::PointXYZ> projection_map;

	std::vector<pcl::PointXYZ> cam_cloud(in_cloud->points.size());
	int x, y, w, h;
	out_cloud->points.clear();
	// loop for point cloud
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
		cam_cloud[i] = TransformPoint(in_cloud->points[i], camera_lidar_tf_);
		int u = int(cam_cloud[i].x * fx_ / cam_cloud[i].z + cx_);
		int v = int(cam_cloud[i].y * fy_ / cam_cloud[i].z + cy_);
		// loop for detected Bounding Boxes
		for (size_t j = 0; j < in_vision_detections->objects.size(); j++) {
			x = in_vision_detections->objects[j].x;
			y = in_vision_detections->objects[j].y;
			w = in_vision_detections->objects[j].width;
			h = in_vision_detections->objects[j].height;
			if ((x <= u) && (u <= x + w) && (y <= v) && (v <= y + h) && (cam_cloud[i].z > 0)) {
				out_cloud->points.push_back(in_cloud->points[i]);
				break;
			}
		}
	}
	// Publish PCl
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*out_cloud, cloud_msg);
	cloud_msg.header = in_cloud_msg->header;
	publisher_fused_cloud_.publish(cloud_msg);
}

void ROSPixelCloudFusionApp::IntrinsicsCallback(const sensor_msgs::CameraInfo &in_message)
{
	image_size_.height = in_message.height;
	image_size_.width = in_message.width;

	camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
		}
	}

	distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
	for (int col = 0; col < 5; col++)
	{
		distortion_coefficients_.at<double>(col) = in_message.D[col];
	}

	fx_ = static_cast<float>(in_message.P[0]);
	fy_ = static_cast<float>(in_message.P[5]);
	cx_ = static_cast<float>(in_message.P[2]);
	cy_ = static_cast<float>(in_message.P[6]);

	intrinsics_subscriber_.shutdown();
	camera_info_ok_ = true;
	ROS_INFO("[%s] CameraIntrinsics obtained.", __APP_NAME__);
}

tf::StampedTransform
ROSPixelCloudFusionApp::FindTransform(const std::string &in_target_frame, const std::string &in_source_frame)
{
	tf::StampedTransform transform;

	camera_lidar_tf_ok_ = false;
	try
	{
		transform_listener_->lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
		camera_lidar_tf_ok_ = true;
		ROS_INFO("[%s] Camera-Lidar TF obtained", __APP_NAME__);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
	}

	return transform;
}

void ROSPixelCloudFusionApp::InitializeROSIo(ros::NodeHandle &in_private_handle)
{
	//get params
	std::string points_src, image_detection_src, camera_info_src, fused_topic_str = "/points_fused";
	std::string name_space_str = ros::this_node::getNamespace();

	ROS_INFO("[%s] This node requires: Registered TF(Lidar-Camera), CameraInfo, Image, and PointCloud.", __APP_NAME__);
	in_private_handle.param<std::string>("points_src", points_src, "/points_raw");
	ROS_INFO("[%s] points_src: %s", __APP_NAME__, points_src.c_str());

	in_private_handle.param<std::string>("image_detection_src", image_detection_src, "/detection/image_detector/objects");
	ROS_INFO("[%s] image_src: %s", __APP_NAME__, image_detection_src.c_str());

	in_private_handle.param<std::string>("camera_info_src", camera_info_src, "/camera_info");
	ROS_INFO("[%s] camera_info_src: %s", __APP_NAME__, camera_info_src.c_str());

	if (name_space_str != "/")
	{
		if (name_space_str.substr(0, 2) == "//")
		{
			name_space_str.erase(name_space_str.begin());
		}
		image_detection_src = name_space_str + image_detection_src;
		fused_topic_str = name_space_str + fused_topic_str;
		camera_info_src = name_space_str + camera_info_src;
	}

	//generate subscribers and sychronizers
	ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, camera_info_src.c_str());
	intrinsics_subscriber_ = in_private_handle.subscribe(camera_info_src,
	                                                     1,
	                                                     &ROSPixelCloudFusionApp::IntrinsicsCallback, this);


	ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, image_detection_src.c_str());
	ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, points_src.c_str());

	publisher_fused_cloud_ = node_handle_.advertise<sensor_msgs::PointCloud2>(fused_topic_str, 1);
	ROS_INFO("[%s] Publishing fused pointcloud in %s", __APP_NAME__, fused_topic_str.c_str());

	cloud_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_, points_src, 1);
    vision_detection_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_, image_detection_src, 1);
    synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10), *cloud_subscriber_, *vision_detection_subscriber_);
	synchronizer_->registerCallback(boost::bind(&ROSPixelCloudFusionApp::SyncedCallback, this, _1, _2));


}


void ROSPixelCloudFusionApp::Run()
{
	ros::NodeHandle private_node_handle("~");
	tf::TransformListener transform_listener;

	transform_listener_ = &transform_listener;

	InitializeROSIo(private_node_handle);

	ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);

	ros::spin();

	ROS_INFO("[%s] END", __APP_NAME__);
}

ROSPixelCloudFusionApp::ROSPixelCloudFusionApp()
{
	camera_lidar_tf_ok_ = false;
	camera_info_ok_ = false;
	processing_ = false;
	image_frame_id_ = "";
}