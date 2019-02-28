#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/segment_differences.h>

int
main (int argc, char** argv)
{
  /*pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());*/

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud <pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud02(new pcl::PointCloud <pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud02_filtered(new pcl::PointCloud <pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_subtracted(new pcl::PointCloud <pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read ("drawer_01.pcd", *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

  reader.read("drawer_02.pcd", *cloud02); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud02->width * cloud02->height
	  << " data points (" << pcl::getFieldsList(*cloud02) << ").";

  // Create the filtering object
  /*pcl::VoxelGrid<pcl::PCLPointCloud2> sor;*/
  pcl::VoxelGrid<pcl::PointXYZ> sor;

  sor.setInputCloud (cloud);
  sor.setLeafSize (0.05f, 0.05f, 0.05f);
  sor.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

  pcl::VoxelGrid<pcl::PointXYZ> sor02;

  sor02.setInputCloud(cloud02);
  sor02.setLeafSize(0.05f, 0.05f, 0.05f);
  sor02.filter(*cloud02_filtered);
  std::cerr << "PointCloud after filtering: " << cloud02_filtered->width * cloud02_filtered->height
	  << " data points (" << pcl::getFieldsList(*cloud02_filtered) << ").";

  /*pcl::PCDWriter writer;
  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);*/

  // Point Cloud subtraction
  pcl::SegmentDifferences<pcl::PointXYZ> seg_diff;

  seg_diff.setTargetCloud(cloud_filtered);
  seg_diff.setInputCloud(cloud02_filtered);
  seg_diff.setDistanceThreshold(0.01);

  seg_diff.segment(*cloud_subtracted);

  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  viewer.showCloud(cloud_subtracted);
  while (!viewer.wasStopped())
  {

  }

  return (0);
}