#include <surface_filter.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <ros/package.h>

SurfaceFilter::SurfaceFilter()
{
  is_data_writed_ = false;
  tf_buffer_ = new tf2_ros::Buffer();
  tf_listener_ = new tf2_ros::TransformListener(*tf_buffer_);
  surface_pointcloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  nh_.getParam(ros::this_node::getName()+"/history_length", history_length_);
  nh_.getParam(ros::this_node::getName()+"/x_surface_size", x_surface_size_);
  nh_.getParam(ros::this_node::getName()+"/y_surface_size", y_surface_size_);
  nh_.getParam(ros::this_node::getName()+"/leaf_size", leaf_size_);
  pointcloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &SurfaceFilter::pointcloud_callback, this);
}

SurfaceFilter::~SurfaceFilter()
{

}

void SurfaceFilter::write_surface_data()
{

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pass.setInputCloud(surface_pointcloud_);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-x_surface_size_/2, x_surface_size_/2);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-y_surface_size_/2, y_surface_size_/2);
  pass.filter(*cloud_filtered);
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud_filtered);
  sor.setLeafSize(leaf_size_,leaf_size_,leaf_size_);
  sor.filter(*cloud_filtered);
  std::string data_file_path = ros::package::getPath("surface_filter") + "/data/surface.pcd";
  pcl::io::savePCDFileASCII(data_file_path, *cloud_filtered);
}

void SurfaceFilter::pointcloud_callback(sensor_msgs::PointCloud2 input_cloud)
{
  if(data_.size() < history_length_)
  {
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf_buffer_->lookupTransform("surface", "camera_rgb_optical_frame",ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
      return;
    }
    tf2::doTransform(input_cloud, input_cloud, transform_stamped);
    data_.push_back(input_cloud);
    pcl::PointCloud<pcl::PointXYZRGB> pcl_pointcloud;
    pcl::fromROSMsg(input_cloud,pcl_pointcloud);
    *surface_pointcloud_ = pcl_pointcloud + *surface_pointcloud_;
    ROS_INFO_STREAM(data_.size() << " pointclouds are processed");
  }
  else
  {
    if(is_data_writed_ == false)
    {
      write_surface_data();
    }
    is_data_writed_ = true;
    ROS_INFO_STREAM("capture finished!!");
  }
}
