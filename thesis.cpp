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
        thesis <filename> [--debug --display --minimum_area=<a> --minimum_number_of_neighbours=<n>]
        thesis (-h | --help)
        thesis --version
    
    Options:
        -h --help   Show this screen.
        --debug   Enable debug mode.
        --display   Display the segmented colored point cloud.
        --minimum_area=<a>  Minimum area of the segmented cluster [default: 0.0].
        --minimum_number_of_neighbours=<n>  Minimum number of neighbours for noise filtration [default: 0].       
        --version   Show version.
)";

int main(int argc, const char** argv)
{
   std::map<std::string, docopt::value> args = docopt::docopt(USAGE, 
                                                  { argv + 1, argv + argc },
                                                  true,               // show help if requested
                                                  "Thesis 1.0");  // version string

    for(auto const& arg : args) {
        std::cout << arg.first <<  arg.second << std::endl;
    }
    
    std::string filename = args["<filename>"].asString();
    bool debug_mode = args["--debug"].asBool();
    bool display_mode = args["--display"].asBool();
    float minimum_area = std::stof(args["--minimum_area"].asString());
    int minimum_number_of_neighbours = std::stoi(args["--minimum_number_of_neighbours"].asString());

    Segmentor seg(filename,minimum_area,minimum_number_of_neighbours,debug_mode);
    int numOfClusters = seg.segment();

    if (display_mode)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Cluster viewer"));
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(seg.coloredCloud()); 
        viewer->setBackgroundColor (0,0,0);
        viewer->addPointCloud(seg.coloredCloud(),rgb,"sample cloud");

        boost::shared_ptr<pcl::visualization::PCLVisualizer> filtered_viewer (new pcl::visualization::PCLVisualizer ("Filtered cluster viewer"));
        filtered_viewer->setBackgroundColor (0,0,0);
        int counter = 0;
        while(counter < seg.filtered_clouds.size())
        {
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(seg.filtered_clouds[counter]);
            std::string name = "Cluster ";
            filtered_viewer->addPointCloud(seg.filtered_clouds[counter],rgb,name + std::to_string(counter));
            counter++;
        }
        

        while (!viewer->wasStopped ())
        {
            viewer->spinOnce(100);
        }
    }

    pcl::io::savePLYFileBinary("data/segmentation_result.ply", *seg.coloredCloud());

    return(0);
}