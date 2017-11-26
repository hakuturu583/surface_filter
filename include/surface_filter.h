#ifndef SURFACE_FILTER_H_INCLUDED
#define SURFACE_FILTER_H_INCLUDED

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

class SurfaceFilter
{
public:
  SurfaceFilter();
  ~SurfaceFilter();
  void write_surface_data();
private:
  ros::NodeHandle nh_;
  ros::Subscriber pointcloud_sub_;
  tf2_ros::Buffer* tf_buffer_;
  tf2_ros::TransformListener* tf_listener_;
  void pointcloud_callback(sensor_msgs::PointCloud2 input_cloud);
  int history_length_;
  std::vector<sensor_msgs::PointCloud2> data_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface_pointcloud_;
  double x_surface_size_,y_surface_size_,leaf_size_;
  bool is_data_writed_;
};
#endif
