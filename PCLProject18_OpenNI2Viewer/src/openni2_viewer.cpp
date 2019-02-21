#include <pcl/common/spring.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <exception>
class SimpleOpenNI2Viewer
{
public:
    pcl::visualization::CloudViewer viewer;
    SimpleOpenNI2Viewer() : viewer("PCL OpenNI2 Viewer")
    {
    }
    void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
        if (!viewer.wasStopped())
        {
            viewer.showCloud(cloud);
        }
    }
    void run()
    {
        try
        {
            pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();
            boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
                boost::bind(&SimpleOpenNI2Viewer::cloudCallback, this, _1);
            interface->registerCallback(f);
            interface->start();
            while (!viewer.wasStopped())
            {
                Sleep(1);
            }
            interface->stop();
        }
        catch (std::exception& e)
        {
            cout << e.what() << endl;
        }
    }
};
int main()
{
    SimpleOpenNI2Viewer v;
    v.run();
    return 0;
}