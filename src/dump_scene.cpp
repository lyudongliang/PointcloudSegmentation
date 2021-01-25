#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/point_cloud.h>
#include <string>
// #include "beginner_tutorials/AddTwoInts.h"
#include "point_cloud_segmentation/PointCloudIO.h"


bool dump_pointcloud(point_cloud_segmentation::PointCloudIO::Request &req, point_cloud_segmentation::PointCloudIO::Response &res)
{
  if (req.file_ending == ".ply")
  {
    pcl::PointCloud<pcl::PointXYZRGBNormal> pclCloud;
    // pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::fromROSMsg(req.cloud_in, pclCloud);
    
    pcl::PLYWriter writer;
    bool binary = false;
    bool use_camera = false;
    writer.write(req.file_path, pclCloud, binary, use_camera);
  }
  else if (req.file_ending == ".pcd")
  {
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::fromROSMsg(req.cloud_in, pclCloud);
    pcl::io::savePCDFileASCII (req.file_path, pclCloud);
  }
  else
  {
    ROS_ERROR("file ending error!");
  }
  
  res.status = true;
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dump_scene_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("dump_scene", dump_pointcloud);
  ROS_INFO("Ready to save dump scene.");
  ros::spin();

  return 0;
}

