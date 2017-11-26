//headers for ros
#include <ros/ros.h>

#include <surface_filter.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ros_ship_map_server_node");
  SurfaceFilter filter;
  ros::spin();
  return 0;
}
