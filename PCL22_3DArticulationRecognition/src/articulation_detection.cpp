// I am aiming to subtract two point clouds and display the subtracted results in one window.
// Include conversion of PointXYZ to PointXYZRGBA

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/segment_differences.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>


typedef pcl::PointXYZRGBA PointC; // Colorful Point
typedef pcl::PointXYZ PointM; // Colorless Point
typedef pcl::PointCloud<PointC> PointCloudC;
typedef pcl::PointCloud<PointM> PointCloudM;

void denoise(PointCloudM::Ptr cloud_in, PointCloudM::Ptr cloud_out);
void passthrough(PointCloudM::Ptr cloud_in, PointCloudM::Ptr cloud_out);

int
main(int argc, char** argv)
{
	//Initialize cloud variables

	PointCloudM::Ptr cloud(new PointCloudM);
	PointCloudM::Ptr cloud_passthrough(new PointCloudM);
	PointCloudM::Ptr cloud_denoised(new PointCloudM);
	PointCloudM::Ptr cloud_filtered(new PointCloudM);

	PointCloudM::Ptr cloud02(new PointCloudM);
	PointCloudM::Ptr cloud02_passthrough(new PointCloudM);
	PointCloudM::Ptr cloud02_denoised(new PointCloudM);
	PointCloudM::Ptr cloud02_filtered(new PointCloudM);

	PointCloudM::Ptr cloud_subtracted(new PointCloudM);


	// Read files
	pcl::PCDReader reader;

	// Read the first cloud data
	reader.read("drawer_01.pcd", *cloud);

	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ")." << endl;

	// Read the second cloud data
	/*reader.read("drawer_03.pcd", *cloud02);

	std::cerr << "PointCloud before filtering: " << cloud02->width * cloud02->height
		<< " data points (" << pcl::getFieldsList(*cloud02) << ")." << endl;*/


	// Filter
	// Pass-through filter
	passthrough(cloud, cloud_passthrough);
	//passthrough(cloud02, cloud02_passthrough);

	// Statistical Outlier Filter
	denoise(cloud_passthrough, cloud_denoised);
	//denoise(cloud02_passthrough, cloud02_denoised);
	std::cout << "Statistical outlier filter done." << std::endl;


	// Create Voxel Grid
	pcl::VoxelGrid<pcl::PointXYZ> voxg;

	voxg.setInputCloud(cloud_denoised);
	voxg.setLeafSize(0.05f, 0.05f, 0.05f);
	voxg.filter(*cloud_filtered);
	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";

	// Region Growing Segmentation
	// Normal estimation
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);

	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(cloud);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);


	// Select the front side of the drawer

	/*pcl::VoxelGrid<pcl::PointXYZ> voxg02;

	voxg02.setInputCloud(cloud02_denoised);
	voxg02.setLeafSize(0.05f, 0.05f, 0.05f);
	voxg02.filter(*cloud02_filtered);
	std::cerr << "PointCloud after filtering: " << cloud02_filtered->width * cloud02_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud02_filtered) << ").";*/

	/*pcl::PCDWriter writer;
	writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered,
		   Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);*/

	// Point Cloud subtraction
	/*pcl::SegmentDifferences<pcl::PointXYZ> seg_diff;

	seg_diff.setTargetCloud(cloud_filtered);
	seg_diff.setInputCloud(cloud02_filtered);
	seg_diff.setDistanceThreshold(0.0025);

	seg_diff.segment(*cloud_subtracted);*/

	// pcl::visualization::CloudViewer viewer("Cloud Viewer");
	pcl::visualization::PCLVisualizer viewer("PCL visualizer");

	// Specify Point Cloud Handler for color
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_subtracted, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_input(cloud02_filtered, 30, 190, 255);

	viewer.addPointCloud(cloud_subtracted, single_color, "cloud segmented");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud segmented");
	viewer.addPointCloud(cloud_filtered, "input cloud 01");
	/*viewer.addPointCloud(cloud02_filtered, single_color_input, "input cloud 02");*/



  while (!viewer.wasStopped())
  {
	  viewer.spinOnce();

  }

  return (0);
}

void denoise(PointCloudM::Ptr cloud_in, PointCloudM::Ptr cloud_out)
{
	// Input cloud, output cloud
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << cloud_in->width * cloud_in->height << std::endl;

	// Create the filtering object
	sor.setInputCloud(cloud_in);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_out);
}

void passthrough(PointCloudM::Ptr cloud_in, PointCloudM::Ptr cloud_out)
{
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << cloud_in->width * cloud_in->height << std::endl;

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_in);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-1.0, 2.0);
	pass.filter(*cloud_out);

	std::cerr << "Clouds after passthrough filter: " << std::endl;
	std::cerr << cloud_out->width * cloud_out->height << std::endl;
}
