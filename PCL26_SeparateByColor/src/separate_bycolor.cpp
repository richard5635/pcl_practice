#include <iostream>
#include <stack>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

bool checkColor(Eigen::Vector3i color);
std::list<int> DColors;
Eigen::Vector3i DetectedColors[100];


int
main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile <pcl::PointXYZRGB>("colored_scene.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}

	// Cloud size setting. Refer to http://www.pointclouds.org/documentation/tutorials/passthrough.php#passthrough
	// Cloud is now in variable cloud. Now separate purely by color.
	// Check how many colors exist inside the cloud.
	int colors[100];
	std::cout << "Pick random color: " << cloud->points[101].getRGBVector3i() << "." << endl;
	std::cout << "Pick random color: " << cloud->points[1672].getRGBVector3i() << "." << endl;
	//for (int i = 0; i < cloud->points.size(); i++)
	//{
	//	// Access each cloud and if they are not red
	//	if (checkColor(cloud->points[i].getRGBVector3i))
	//	{
	//		DColors.push_back(cloud->points[i].getRGBVector3i);
	//	}
	//}

	//int colors = DColors.size();

	//// Prepare cloud variable with heigh equal to number of colors.
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sep(new pcl::PointCloud<pcl::PointXYZ>);
	//cloud_sep->width = 5;
	//cloud_sep->height = (unsigned int)colors;

	return 0;
}

bool checkColor(Eigen::Vector3i color)
{
	// Compare the checked color with existing color.
	return false;
}

bool isRed(Eigen::Vector3i color)
{
	return false;
}