#include "segmentor.hpp"
#include <pcl/visualization/pcl_visualizer.h>

int main()
{

    Segmentor seg("region_growing_tutorial.pcd");
    int numOfClusters = seg.segment();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Cluster viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(seg.coloredCloud()); 
    viewer->setBackgroundColor (0,0,0);
    viewer->addPointCloud(seg.coloredCloud(),rgb,"sample cloud");

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce(100);
    }

    return(0);
}