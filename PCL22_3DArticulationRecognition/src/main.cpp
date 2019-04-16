// I am aiming to subtract two point clouds and display the subtracted results in one window.
// Include conversion of PointXYZ to PointXYZRGBA

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/segment_differences.h>

typedef pcl::PointXYZRGBA PointC;
typedef pcl::PointXYZ PointM;
typedef pcl::PointCloud<PointC> PointCloudC;
typedef pcl::PointCloud<PointM> PointCloudM;

int
main(int argc, char** argv)
{
	/*pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
	pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());*/

	PointCloudM::Ptr cloud(new PointCloudM);
	PointCloudM::Ptr cloud_filtered(new PointCloudM);

	PointCloudM::Ptr cloud02(new PointCloudM);
	PointCloudM::Ptr cloud02_filtered(new PointCloudM);

	PointCloudM::Ptr cloud_subtracted(new PointCloudM);


	// Fill in the cloud data
	pcl::PCDReader reader;

	// Replace the path below with the path where you saved your file
	reader.read("drawer_01.pcd", *cloud); // Remember to download the file first!

	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").";

	reader.read("drawer_03.pcd", *cloud02); // Remember to download the file first!

	std::cerr << "PointCloud before filtering: " << cloud02->width * cloud02->height
		<< " data points (" << pcl::getFieldsList(*cloud02) << ").";

	// Pass through filter, to filter the background..?

	// Create Voxel Grid
	pcl::VoxelGrid<pcl::PointXYZ> sor;

	sor.setInputCloud(cloud);
	sor.setLeafSize(0.05f, 0.05f, 0.05f);
	sor.filter(*cloud_filtered);
	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";

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
	seg_diff.setDistanceThreshold(0.0025);

	seg_diff.segment(*cloud_subtracted);

	// pcl::visualization::CloudViewer viewer("Cloud Viewer");
	pcl::visualization::PCLVisualizer viewer("PCL visualizer");

	// Specify Point Cloud Handler for color
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_subtracted, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_input(cloud02_filtered, 30, 190, 255);

	viewer.addPointCloud(cloud_subtracted, single_color, "cloud segmented");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud segmented");
	viewer.addPointCloud(cloud_filtered, "input cloud 01");
	viewer.addPointCloud(cloud02_filtered, single_color_input, "input cloud 02");

  while (!viewer.wasStopped())
  {
	  viewer.spinOnce();

  }

  return (0);
}