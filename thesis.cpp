#include "segmentor.hpp"
#include "docopt/docopt.h"
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_density.h>


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

    // for(auto const& arg : args) {
    //     std::cout << arg.first <<  arg.second << std::endl;
    // }
    
    std::string filename = args["<filename>"].asString();
    bool debug_mode = args["--debug"].asBool();
    bool display_mode = args["--display"].asBool();
    float minimum_area = std::stof(args["--minimum_area"].asString());
    int minimum_number_of_neighbours = std::stoi(args["--minimum_number_of_neighbours"].asString());


    SegmentorOptions opt;
    opt.DISPLAY_MODE = display_mode;
    opt.DEBUG_MODE = debug_mode;
    opt.MINIMUM_AREA = minimum_area;
    opt.NOISE_THRESHOLD = minimum_number_of_neighbours;
    opt.SAVE_CLUSTERS_OF_INTEREST = false;
    opt.SAVE_EDGES_FOR_CLUSTERS_OF_INTEREST = false;
    opt.SAVE_PROJECTED_CLUSTERS_OF_INTEREST = false;

    Segmentor seg(filename,opt);
    int numOfClusters = seg.segment();

    // Concatenate projected clouds
    std::cout<<"Generating z-value histogram"<<std::endl;
    pcl::octree::OctreePointCloudDensity<pcl::PointXYZRGB> octree(0.1f);
    octree.setInputCloud(seg.projected_cloud);
    octree.addPointsFromInputCloud();
    
    float min_z=seg.projected_cloud->points[0].z;
    float max_z=seg.projected_cloud->points[0].z;
    for(size_t i=0;i<seg.projected_cloud->points.size();i++)
    {
        if(seg.projected_cloud->points[i].z<min_z)
        {
            min_z = seg.projected_cloud->points[i].z;
        }else if(seg.projected_cloud->points[i].z>max_z)
        {
            max_z = seg.projected_cloud->points[i].z;
        }
    }

    std::vector<std::pair<double,double>> data;
    float step = 0.1f;
    int steps = (max_z-min_z)/step;
    data.resize(steps);


    for(float z=min_z; z<max_z; z += step)
    {
        pcl::PointXYZRGB p;
        p.x=0;
        p.y=0;
        p.z=z;
        int density = octree.getVoxelDensityAtPoint(p);
        data.push_back(std::make_pair(z,density));
    }

    pcl::visualization::PCLPlotter *plotter = new pcl::visualization::PCLPlotter("z histogram");
    plotter->addPlotData(data);
    plotter->plot();

    std::cout<<"Done"<<std::endl;

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
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_cluster(seg.filtered_clouds[counter]);
            std::string name = "Cluster ";
            filtered_viewer->addPointCloud(seg.filtered_clouds[counter],rgb_cluster,name + std::to_string(counter));
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