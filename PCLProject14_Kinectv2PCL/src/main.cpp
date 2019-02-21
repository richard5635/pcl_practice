
// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include "kinect2_grabber.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZRGBA PointType;


int main( int argc, char* argv[] )
{
	

    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer( "Point Cloud Viewer" ) );
    viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0, 0.0, 0.0 );

    // Point Cloud
    pcl::PointCloud<PointType>::ConstPtr cloud;

    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;
    boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =
        [&cloud, &mutex]( const pcl::PointCloud<PointType>::ConstPtr& ptr ){
            boost::mutex::scoped_lock lock( mutex );

            /* Point Cloud Processing */

            cloud = ptr->makeShared();
        };

    // Kinect2Grabber
    boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );

	// Keyboard Callback function
	boost::function<void( const pcl::visualization::KeyboardEvent& )> keyboard_function =
		[&cloud, &mutex](const pcl::visualization::KeyboardEvent& event) {
		// Save Point Cloud to PCD file when Pressed Space Key
		if (event.getKeyCode() == VK_SPACE && event.keyDown()) {
			boost::mutex::scoped_try_lock lock(mutex);
			if (lock.owns_lock()) {
				// pcl::io::savePCDFile("captured_cloud.pcd", *cloud, true); //Binary format
				pcl::io::savePCDFile("captured_cloud.pcd", *cloud, false); //ASCII format
			}
		}
	};

	// Register callback function
	viewer->registerKeyboardCallback(keyboard_function);

    // Start Grabber
    grabber->start();

	char c;
	bool save_one;

	save_one = false;

    while( !viewer->wasStopped() ){
        // Update Viewer
        viewer->spinOnce();

        boost::mutex::scoped_try_lock lock( mutex );
        if( lock.owns_lock() && cloud ){
			if (cloud->size() != 0) {
				/* Processing Point Cloud */

				// Update Point Cloud
				if (!viewer->updatePointCloud(cloud, "cloud")) {
					viewer->addPointCloud(cloud, "cloud");
					viewer->resetCameraViewpoint("cloud");
				}
			}
        }
    }

    // Stop Grabber
    grabber->stop();

    return 0;
}
