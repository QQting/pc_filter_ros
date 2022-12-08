#include "ros/ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <memory>

// Dynamic_reconfigure
#include "dynamic_reconfigure/server.h"
#include "pc_filter_ros/PcFilterConfig.h"

ros::Publisher filtered_pc_pub;
ros::NodeHandle* n;

double min_limit_x_;
double max_limit_x_;
double min_limit_y_;
double max_limit_y_;
double min_limit_z_;
double max_limit_z_;
bool reversed_filter_;

std::string filtered_frame_id_;
std::string observed_frame_id_;
std::string input_pc_topic_;
std::string output_pc_topic_;

void filterCallback(const sensor_msgs::PointCloud2ConstPtr& sensor_message_pc)
{  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xy(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::fromROSMsg(*sensor_message_pc,*cloud_in);
  pcl::PassThrough<pcl::PointXYZ> pass;

  pass.setFilterLimitsNegative(reversed_filter_);
  
  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (min_limit_x_, max_limit_x_);
  pass.filter(*cloud_filtered_x);

  pass.setInputCloud(cloud_filtered_x);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (min_limit_y_, max_limit_y_);
  pass.filter(*cloud_filtered_xy);

  pass.setInputCloud(cloud_filtered_xy);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (min_limit_z_, max_limit_z_);
  pass.filter(*cloud_filtered_xyz);


  sensor_msgs::PointCloud2 cloud_out_rosmsg;
  cloud_out_rosmsg.header.frame_id = filtered_frame_id_;
  pcl::toROSMsg(*cloud_filtered_xyz, cloud_out_rosmsg);
  filtered_pc_pub.publish(cloud_out_rosmsg);
}

void reconfigureCallback(pc_filter_ros::PcFilterConfig &config, uint32_t level)
{
  ROS_DEBUG("Reconfigure Request: x=[%.2lf, %.2lf], y=[%.2lf, %.2lf],  z=[%.2lf, %.2lf], negative=%d", 
            config.min_limit_x,
            config.max_limit_x,
            config.min_limit_y,
            config.max_limit_y,
            config.min_limit_z,
            config.max_limit_z,
            config.reversed_filter
          );
  
  min_limit_x_ = config.min_limit_x;
  max_limit_x_ = config.max_limit_x;
  min_limit_y_ = config.min_limit_y;
  max_limit_y_ = config.max_limit_y;
  min_limit_z_ = config.min_limit_z;
  max_limit_z_ = config.max_limit_z;
  reversed_filter_ = config.reversed_filter;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_filter_server");
  ros::NodeHandle nh("~");

  nh.getParam("min_limit_x", min_limit_x_);
  nh.getParam("max_limit_x", max_limit_x_);
  nh.getParam("min_limit_y", min_limit_y_);
  nh.getParam("max_limit_y", max_limit_y_);
  nh.getParam("min_limit_z", min_limit_z_);
  nh.getParam("max_limit_z", max_limit_z_);
  nh.getParam("reversed_filter", reversed_filter_);
  nh.getParam("observed_frame_id", observed_frame_id_);
  nh.getParam("filtered_frame_id", filtered_frame_id_);
  nh.getParam("input_pc_topic", input_pc_topic_);
  nh.getParam("output_pc_topic", output_pc_topic_);

  ROS_INFO_STREAM("Listening on this input pc topic " << input_pc_topic_);
  ROS_INFO_STREAM("Listening on this output pc topic " << output_pc_topic_);
  ROS_INFO_STREAM("Listening on this observed_frame_id " << observed_frame_id_);
  ROS_INFO_STREAM("Listening on this filtered_frame_id " << filtered_frame_id_);
  ROS_INFO_STREAM("Dimensions for filtered scene are: (" << min_limit_x_ << ", " << max_limit_x_ << ") (" << min_limit_y_ << ", " << max_limit_y_ << ") (" << min_limit_z_ << ", " << max_limit_z_ << ")");

  dynamic_reconfigure::Server<pc_filter_ros::PcFilterConfig> server;
  dynamic_reconfigure::Server<pc_filter_ros::PcFilterConfig>::CallbackType cb = boost::bind(&reconfigureCallback, _1, _2);
  server.setCallback(cb);

  ros::Subscriber original_pc_sub = nh.subscribe(input_pc_topic_, 1, filterCallback);
  filtered_pc_pub = nh.advertise<sensor_msgs::PointCloud2>(output_pc_topic_, 1);

  ros::spin();
  return 0;
}

