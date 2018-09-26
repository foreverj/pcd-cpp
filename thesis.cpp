#include "segmentor.hpp"
#include "docopt/docopt.h"
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>


static const char USAGE[]=
R"(Thesis
    Usage:
        thesis <filename>
        thesis (-h | --help)
        thesis --version
    
    Options:
        -h --help   Show this screen.
        --version   Show version.
)";

int processingFile(std::string filename)
{
    Segmentor seg(filename);
    int numOfClusters = seg.segment();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Cluster viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(seg.coloredCloud()); 
    viewer->setBackgroundColor (0,0,0);
    viewer->addPointCloud(seg.coloredCloud(),rgb,"sample cloud");
    pcl::io::savePLYFileBinary("data/segmentation_result.ply", *seg.coloredCloud());

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce(100);
    }

    return 0;
}

int main(int argc, const char** argv)
{
   std::map<std::string, docopt::value> args = docopt::docopt(USAGE, 
                                                  { argv + 1, argv + argc },
                                                  true,               // show help if requested
                                                  "Thesis 1.0");  // version string


    std::string filename = args["<filename>"].asString();

    processingFile(filename);

    return(0);
}