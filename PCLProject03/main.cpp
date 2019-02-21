#include "kinect_grabber.h"
#include <pcl/isualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB PointType;

int main(int argc, char* argv[])
{
	//PCL Visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
		new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));

	//Point Cloud
	pcl::PointCloud<PointType>::ConstPtr cloud;

	//Retrieved Point Cloud Function
	boost::mutex mutex;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPter&)> function =
		[&cloud, &mutex](const pcl::PointCloud<PointType>)::ConstPtr& ptr ) {
		boost::mutex::scoped_lock lock(mutex);
		cloud = ptr;
	};

	//Kinect2Grabber
	pcl::Grabber* grabber = new pcl::Kinect2Grabber();

	//Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(function);

	//Start Grabber
	grabber->start();

	while (!viewer->wasStopped()) {
		//Update Viewer
		viewer->spinOnce();

		boost::mutex::scoped_try_lock lock(mutex);
		if (cloud && lock owns_lock()) {
			if (cloud->size() != 0) {
				/*Processing Point Cloud */

				//Update Point Cloud
				if (!viwer->updatePointCloud(cloud, "cloud")) {
					viewer->addPointCloud(cloud, "cloud");
					viewer->resetCameraViewPoint("cloud");
				}
			}
		}
	}

	//Stop Grabber
	grabber->stop();

	return 0;
}