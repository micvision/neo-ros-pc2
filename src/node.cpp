
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

typedef dynamic_reconfigure::Server<neo_ros_pc2::FilterConfig> FilterConfigServer;

neo_filter::Config filter_config;

void callback(neo_ros_pc2::FilterConfig &config, uint32_t level) {
    filter_config.MedianFilter = config.median_filter_;
    filter_config.MedianFilterWindowsSize = config.median_filter_half_windows_size_ * 2 + 1;
    filter_config.ClosedPointFilter = config.close_point_filter_;
    filter_config.ClosePointDistance = config.close_point_distance_;

    ROS_DEBUG("Reconfigure Request:");
    ROS_DEBUG("  median_filter: %s", config.median_filter_ ? "True" : "False");
    ROS_DEBUG("  median_filter_windows_size_: %d", config.median_filter_half_windows_size_);
    ROS_DEBUG("  close_point_filter: %s", config.close_point_filter_ ? "True" : "False");
    ROS_DEBUG("  close_point_distance: %d", config.close_point_distance_);

}


void median_filter(pcl::PointCloud<pcl::PointXYZ> *pointcloud) {
    ROS_DEBUG("median filter");

}

void publish_scan(ros::Publisher *pub,
                  const neo::scan *scan, std::string frame_id)
{
    pcl::PointCloud <pcl::PointXYZ> cloud;
    pcl::PointCloud <pcl::PointXYZ> cloud_polar;
    sensor_msgs::PointCloud2 cloud_msg;
    ros::Time ros_time_now = ros::Time::now();


    float angle;
    int32_t range;
    float x;
    float y;
    int i = 0;

    cloud.height = 1;
    cloud.width = scan->samples.size();
    cloud.points.resize(cloud.width * cloud.height);

    cloud_polar.height = 1;
    cloud_polar.width = cloud.width;
    cloud_polar.resize(cloud_polar.width * cloud_polar.height);

    for (const neo::sample& sample : scan->samples)
    {
        range = sample.distance;
        if (filter_config.ClosedPointFilter) {
            if (range < filter_config.ClosePointDistance)
                continue;
        }
        angle = ((float)sample.angle / 1000); //millidegrees to degrees

        //Polar to Cartesian Conversion
        x = (range * cos(DEG2RAD(angle))) / 100;
        y = (range * sin(DEG2RAD(angle))) / 100;

        cloud.points[i].x = x;
        cloud.points[i].y = y;

        cloud_polar.points[i].x = float(range);
        cloud_polar.points[i].y = angle;
        i++;
    }
    cloud.width = i;
    cloud_polar.width = i;
    cloud.points.resize(i * cloud.height);
    cloud_polar.points.resize(i * cloud_polar.height);
    if (filter_config.MedianFilter)
        median_filter(&cloud_polar);

    //Convert pcl PC to ROS PC2
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.header.stamp = ros_time_now;

    ROS_DEBUG("Publishing a full scan");
    ROS_DEBUG("Scan number: %d", i);
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
        //Grab Full Scan
        const neo::scan scan = device.get_scan();

        publish_scan(&scan_pub, &scan, frame_id);

        ros::spinOnce();
    }

    //Stop Scanning & Destroy Driver
    device.stop_scanning();
    device.set_motor_speed(0);
} catch (const neo::device_error& e) {
      ROS_ERROR_STREAM("Error: " << e.what() << std::endl);
}
