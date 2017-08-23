
/*The MIT License (MIT)
 *
 * Copyright (c) 2017, Scanse, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "ros/ros.h"
#include <iostream>
#include <pcl/point_types.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <neo/neo.hpp>

#include <dynamic_reconfigure/server.h>
#include <neo_ros_pc2/FilterConfig.h>
#include <neo_ros_pc2/neo_filter.h>
#include <neo_ros_pc2/line_param.h>
#include <neo_ros_pc2/datatypes.h>
#include <neo_ros_pc2/line_extraction.h>


typedef dynamic_reconfigure::Server<neo_ros_pc2::FilterConfig> FilterConfigServer;

neo_filter::Config filter_config;
line_param::Config line_param_config;

void callback(const neo_ros_pc2::FilterConfig &config, uint32_t level) {
    // filter config
    filter_config.MedianFilter = config.median_filter_;
    filter_config.MedianFilterWindowsSize = config.median_filter_half_windows_size_ * 2 + 1;
    filter_config.ClosedPointFilter = config.close_point_filter_;
    filter_config.ClosePointDistance = config.close_point_distance_;
    filter_config.MaxDistance = config.max_distance_;

    // line parameter config
    line_param_config.Interpolation = config.interpolation_;
    line_param_config.LargestSquareDistanceOfLine = static_cast<float>(config.largest_square_distance_of_line_);
    line_param_config.CollinearityParam = static_cast<float>(config.collinearity_param_);
    line_param_config.InterpolationPointNum = config.interpolation_point_num_;
    line_param_config.NumPointsOfLine = config.num_points_of_line_;

    ROS_DEBUG("Reconfigure Request:");
    ROS_DEBUG("  median_filter: %s", config.median_filter_ ? "True" : "False");
    ROS_DEBUG("  median_filter_windows_size_: %d", config.median_filter_half_windows_size_);
    ROS_DEBUG("  close_point_filter: %s", config.close_point_filter_ ? "True" : "False");
    ROS_DEBUG("  close_point_distance: %d", config.close_point_distance_);

}


float median_value(const PointCloudXY::Ptr pointcloud) {
    float value;
    int large, equal, less;

    for (unsigned int i = 0; i < pointcloud->points.size(); i++) {
        large = 0; equal = 0; less = 0;
        value = pointcloud->points[i].y;
        for (unsigned int j = 0; j < pointcloud->points.size(); j++) {
            if (i == j) continue;
            if (pointcloud->points[j].y == value) equal++;
            else if (pointcloud->points[j].y > value) large++;
            else less++;
        }
        if (large > less + equal || less > large + equal)
            continue;
        else
            break;
    }
    return value;
}


void median_filter(const PointCloudXY::Ptr pointcloud) {
    int half_windows_size;
    //pcl::PointCloud<pcl::PointXYZ> temp_pc;
    PointCloudXY::Ptr temp_pc(new PointCloudXY);

    half_windows_size = std::floor(filter_config.MedianFilterWindowsSize / 2);
    pcl::copyPointCloud(*pointcloud, *temp_pc);

    for (int i = 0; i < (int)temp_pc->points.size(); i++) {
        // construct the fix filter window
        PointCloudXY::Ptr fixed_windows_size_pc(new PointCloudXY);
        for (int j = i - half_windows_size; j <= i + half_windows_size; j++) {
            if (j < 0)
                fixed_windows_size_pc->push_back(temp_pc->points[j+temp_pc->points.size()]);
            else if (j >= (int)temp_pc->points.size())
                fixed_windows_size_pc->push_back(temp_pc->points[j-temp_pc->points.size()]);
            else
                fixed_windows_size_pc->push_back(temp_pc->points[j]);
        }

        pointcloud->points[i].x = temp_pc->points[i].x;
        pointcloud->points[i].y = median_value(fixed_windows_size_pc);
        // std::cout << i << ": " << pointcloud->points[i].x << " "
        //     << pointcloud->points[i].y << " "
        //     << "origin point: "
        //     << temp_pc->points[i].x << " "
        //     << temp_pc->points[i].y << std::endl;
    }

}

void publish_scan(ros::Publisher *pub,
                  const neo::scan *scan, std::string frame_id)
{
    PointCloudXYZ::Ptr pub_cloud(new PointCloudXYZ);

    PointCloudXY::Ptr polar_cloud(new PointCloudXY);
    PointCloudXY::Ptr cartesian_cloud(new PointCloudXY);

    sensor_msgs::PointCloud2 cloud_msg;
    // ros::Time ros_time_now = ros::Time::now();


    float angle;
    int32_t range;
    float x;
    float y;
    int i = 0;

    polar_cloud->height = 1;
    polar_cloud->width = scan->samples.size();
    polar_cloud->resize(polar_cloud->width * polar_cloud->height);

    for (const neo::sample& sample : scan->samples)
    {
        range = sample.distance;
        angle = ((float)sample.angle / 1000); //millidegrees to degrees
        if (filter_config.ClosedPointFilter) {
            if (range < filter_config.ClosePointDistance || range > filter_config.MaxDistance)
                continue;
        }

        polar_cloud->points[i].y = float(range);
        polar_cloud->points[i].x = angle;

        if (i >= 1) {
            if (angle == polar_cloud->points[i-1].x) continue;
        }
        i++;
    }

    if (i <= 1) return;
    polar_cloud->width = i;
    polar_cloud->points.resize(i * polar_cloud->height);

    if (filter_config.MedianFilter)
        median_filter(polar_cloud);

    cartesian_cloud->height = 1;
    cartesian_cloud->width = i;
    cartesian_cloud->points.resize(cartesian_cloud->width * cartesian_cloud->height);

    for (size_t index = 0; index < polar_cloud->points.size(); index++) {
        angle = polar_cloud->points[index].x;
        range = polar_cloud->points[index].y;

        // Polar to Cartesian Conversion
        x = ((float)range * cos(DEG2RAD(angle)));
        y = ((float)range * sin(DEG2RAD(angle)));
        cartesian_cloud->points[index].x = x;
        cartesian_cloud->points[index].y = y;
    }

	if (line_param_config.Interpolation) {
		/*
		 *PointCloudXYZ::Ptr interpolation_cloud(new PointCloudXYZ);
		 *sensor_msgs::PointCloud2 interpolation_cloud_msg;
		 */
		LineExtraction line_extraction(*cartesian_cloud);

		for (size_t ii = 0; ii < line_extraction.point_cloud_interpolation_.points.size(); ii++) {
			line_extraction.point_cloud_interpolation_.points[ii].x /= 100;
			line_extraction.point_cloud_interpolation_.points[ii].y /= 100;
		}

		pcl::copyPointCloud(line_extraction.point_cloud_interpolation_, *pub_cloud);
		/*
		 *pcl::toROSMsg(*interpolation_cloud, interpolation_cloud_msg);
		 *interpolation_cloud_msg.header.frame_id = frame_id;
		 *interpolation_cloud_msg.header.stamp = ros::Time::now();
		 */

	} else {
		for (size_t ii = 0; ii < cartesian_cloud->points.size(); ii++) {
			cartesian_cloud->points[ii].x /= 100;
			cartesian_cloud->points[ii].y /= 100;
		}

		pcl::copyPointCloud(*cartesian_cloud, *pub_cloud);
	}
	//Convert pcl PC to ROS PC2
	pcl::toROSMsg(*pub_cloud, cloud_msg);
	cloud_msg.header.frame_id = frame_id;
	cloud_msg.header.stamp = ros::Time::now();

	ROS_DEBUG("Publishing a full scan");
	pub->publish(cloud_msg);
}



int main(int argc, char *argv[]) try
{
    //Initialize Node and handles
    ros::init(argc, argv, "neo_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //Get Serial Parameters
    std::string serial_port;
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    int serial_baudrate;
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200);

    //Get Scanner Parameters
    int rotation_speed;
    nh_private.param<int>("rotation_speed", rotation_speed, 5);

    //Get frame id Parameters
    std::string frame_id;
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");

    //Setup Publisher
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::PointCloud2>("pc2", 1000);
    //ros::Publisher interpolation_pub = nh.advertise<sensor_msgs::PointCloud2>("interpolation", 1000);

    //Create Neo Driver Object
    neo::neo device{serial_port.c_str()};
    ROS_INFO("Device connect successful!");

    //Send Rotation Speed
    device.set_motor_speed(rotation_speed);

    ROS_INFO("expected rotation frequency: %d (Hz)", rotation_speed);

    //Start Scan
    device.start_scanning();

    // dynamic_reconfigure server
    FilterConfigServer server;
    FilterConfigServer::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    while (ros::ok())
    {
        const neo::scan scan = device.get_scan();
        //Grab Full Scan

        publish_scan(&scan_pub, &scan, frame_id);

        ros::spinOnce();
    }

    //Stop Scanning & Destroy Driver
    device.stop_scanning();
    device.set_motor_speed(0);
} catch (const neo::device_error& e) {
      ROS_ERROR_STREAM("Error: " << e.what() << std::endl);
}
