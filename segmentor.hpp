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
#include <pcl/filters/radius_outlier_removal.h>


struct SegmentorOptions{
    bool DEBUG_MODE = false; // If debug log is enabled.
    bool SAVE_CLUSTERS_OF_INTEREST = true; 
    bool SAVE_PROJECTED_CLUSTERS_OF_INTEREST = true;
    bool SAVE_EDGES_FOR_CLUSTERS_OF_INTEREST = true;

    float MINIMUM_AREA = 0.0f; // The minimum area required for a cluster to be considered as a celling, floor or wall candidate.
    int NOISE_THRESHOLD = 0; // The number of points within a reasonable proximity of the point for it not to be considered as a noise point.
    float NOISE_RADIUS = 0.1f; // The searching radius of the noise removal filter.

    // Parameters for normals calculation
    int NORMALS_KSEARCH = 50;

    // Parameters for regional growing algorithm
    int REGIONAL_GROWING_MIN_CLUSTER_SIZE = 50;
    int REGIONAL_GROWING_MAX_CLUSTER_SIZE = 1000000;
    int REGIONAL_GROWING_NUMBER_OF_NEIGHBOURS = 30;
    float REGIONAL_GROWING_SMOOTHNESS_THRESHOLD = 3.0 / 180.0 * M_PI;
    float REGIONAL_GROWING_CURVATURE_THRESHOLD = 1.0f;

    // Parameters for SAC algorithm
    float SAC_DISTANCE_THRESHOLD = 0.2f;


};


class Segmentor{
    public:
        Segmentor(std::string filename,SegmentorOptions options);
        SegmentorOptions options;
        int segment ();
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr coloredCloud();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        std::vector <pcl::PointCloud<pcl::PointXYZRGB>::Ptr> filtered_clouds;
        //std::vector <pcl::RGB> filtered_clouds_color_map;
        std::vector <pcl::ModelCoefficients::Ptr> cluster_coefficients;
        std::vector <pcl::PointIndices> clusters;
    private:
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        std::vector<pcl::RGB> color_map;
        void projectPointsAndSavePly(std::vector <pcl::ModelCoefficients::Ptr> cluster_coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::vector <pcl::PointIndices> clusters);
        void extractPointCloudsFromCoefficients(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ> &cluster_cloud,pcl::PointIndices::Ptr cluster);
        float calculateAreaPolygon(const pcl::PointCloud<pcl::PointXYZ> &polygon);
};

#endif