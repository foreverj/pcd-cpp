#include "segmentor.hpp"
#include "docopt/docopt.h"
#include "boundary.hpp"
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
    opt.SAVE_EDGES_FOR_CLUSTERS_OF_INTEREST = true;
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

    std::cout<<"Done"<<std::endl;

    std::cout<<"Finding ceilings and floors ..."<<std::endl;

    double z_critical_values[10];
    int z_counter = 0;
    for(int i=0; i<data.size()-1;i++)
    {
        double z_i = data[i].first;
        double d_i = data[i].second;

        double z_i1 = data[i+1].first;
        double d_i1 = data[i+1].second;

        double criterion = abs(d_i-d_i1)/d_i;

        if(criterion > 1.0)
        {
            z_critical_values[z_counter] = z_i1;
            z_counter++;
            if (z_counter > 9)
            {
                throw("size for critical z is too small");
            }
            std::cout<<"Z value: "<<z_i1<<std::endl;
        }
    }

    // Brutal Force Start
    double floor_z = z_critical_values[0];
    double ceiling1_z = z_critical_values[2];
    double ceiling2_z = z_critical_values[3];

    pcl::PointCloud<pcl::PointXYZRGB> contourCloud;

    //Iterate over all filtered clusters
    for(int i=0;i<seg.filtered_clouds.size();i++)
    {
        Eigen::Vector4f centroid; 
        pcl::compute3DCentroid(*seg.filtered_clouds[i],centroid);
        std::cout<<centroid[2]<<std::endl;
        if(centroid[2]>=floor_z-step/2 && centroid[2]<=floor_z+step/2)
        {
            std::cout<<seg.filtered_clouds[i]->points.size()<<std::endl;
            pcl::io::savePLYFile("floor_"+std::to_string(i)+"_edges.ply",*seg.filtered_clouds[i],false);
            contourCloud += *seg.filtered_clouds[i];
        }

        if(centroid[2]>=ceiling1_z-step/2 && centroid[2]<=ceiling2_z+step/2)
        {
            std::cout<<seg.filtered_clouds[i]->points.size()<<std::endl;
            pcl::io::savePLYFile("ceiling_"+std::to_string(i)+"_edges.ply",*seg.filtered_clouds[i],false);
            contourCloud += *seg.filtered_clouds[i];
        }
    }

    // z=0
    pcl::ModelCoefficients::Ptr xyPlaneCoefficients (new pcl::ModelCoefficients());
    xyPlaneCoefficients->values.resize(4);
    xyPlaneCoefficients->values[0]=0;
    xyPlaneCoefficients->values[1]=0;
    xyPlaneCoefficients->values[2]=1.0;
    xyPlaneCoefficients->values[3]=0.0-floor_z;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xy_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr contourCloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    contourCloud_ptr->points.resize(contourCloud.points.size());
    for(size_t i=0;i<contourCloud.points.size();i++)
    {
        contourCloud_ptr->points[i].x = contourCloud.points[i].x;
        contourCloud_ptr->points[i].y = contourCloud.points[i].y;
        contourCloud_ptr->points[i].z = contourCloud.points[i].z;
    }


    pcl::ProjectInliers<pcl::PointXYZRGB> xy_proj;
    xy_proj.setModelType(pcl::SACMODEL_PLANE);
    xy_proj.setModelCoefficients(xyPlaneCoefficients);
    xy_proj.setInputCloud (contourCloud_ptr);
    xy_proj.filter (*xy_pc);

    pcl::PointCloud<pcl::PointXYZ>::Ptr xy_pc_xyz (new pcl::PointCloud<pcl::PointXYZ>);

    xy_pc_xyz->points.resize(xy_pc->points.size());
    for (size_t i = 0; i < xy_pc->points.size(); i++) {
        xy_pc_xyz->points[i].x = xy_pc->points[i].x;
        xy_pc_xyz->points[i].y = xy_pc->points[i].y;
        xy_pc_xyz->points[i].z = xy_pc->points[i].z;
    }

    pcl::io::savePLYFile("contour_points.ply",*xy_pc_xyz,false);

    BoundaryProcessor bp(*xy_pc_xyz,*xyPlaneCoefficients,debug_mode);
    bp.processData();
    pcl::io::savePLYFile("contour_edges.ply",bp.converted_points,false);

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