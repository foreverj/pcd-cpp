#ifndef _SEGMENTOR_H
#define _SEGMENTOR_H

#include <fstream>
#include <iostream>

#include <stdexcept>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>



class Segmentor{
    public:
        Segmentor(std::string filename);
        int segment ();
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr coloredCloud();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        std::vector <pcl::ModelCoefficients::Ptr> cluster_coefficients;
        std::vector <pcl::PointIndices> clusters;
    private:
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        void projectPointsAndSavePly(std::vector <pcl::ModelCoefficients::Ptr> cluster_coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::vector <pcl::PointIndices> clusters);
        void extractPointCloudsFromCoefficients(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ> &cluster_cloud,pcl::PointIndices::Ptr cluster);
};

#endif