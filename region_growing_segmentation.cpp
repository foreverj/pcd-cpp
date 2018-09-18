#include <fstream>
#include <iostream>
#include <vector>
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

int saveCoefficients(std::string filename,std::vector <pcl::ModelCoefficients::Ptr> cluster_coefficients)
{
  std::string header="cluster number,coefficient 0,coefficient 1,coefficient 2,coefficient 3\n";
  std::ofstream out(filename);
  out << header;

  int counter=0;
  while(counter<cluster_coefficients.size())
  {
    out<<""<<counter<<","
                <<cluster_coefficients[counter]->values[0]<<","
                <<cluster_coefficients[counter]->values[1]<<","
                <<cluster_coefficients[counter]->values[2]<<","
                <<cluster_coefficients[counter]->values[3]<<"\n";
    counter++;
  }
  out.close();
  return 0;
}

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("region_growing_tutorial.pcd", *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
  std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;
  int counter = 0;
  // while (counter < clusters[0].indices.size ())
  // {
  //   std::cout << clusters[0].indices[counter] << ", ";
  //   counter++;
  //   if (counter % 10 == 0)
  //     std::cout << std::endl;
  // }
  // std::cout << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::io::savePCDFile("colored_cloud.pcd",*colored_cloud);
  pcl::io::savePLYFile("colored_cloud.ply",*colored_cloud,false);

  //Fit plane equations to each cluster using RANSAC
  counter = 0;

  std::vector <pcl::ModelCoefficients::Ptr> cluster_coefficients;
  cluster_coefficients.resize(clusters.size());

  while(counter<clusters.size())
  {
    std::cout <<"Start processing cluster " << counter <<std::endl;

    //Cluster point cloud extraction variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr cluster (new pcl::PointIndices);

    int index_counter = 0;
    while(index_counter<clusters[counter].indices.size())
    {
      cluster->indices.resize(clusters[counter].indices.size());
      cluster->indices[index_counter]=clusters[counter].indices[index_counter];
      index_counter++;
    }

    extract.setInputCloud (cloud);
    extract.setIndices (cluster);
    extract.setNegative (false);
    extract.filter (*cluster_cloud);

    pcl::io::savePLYFile("cluster"+std::to_string(counter)+".ply",*cluster_cloud,false);

    //SAC segmentation variables
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    
    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (1);
    seg.setInputCloud (cluster_cloud);
    seg.segment (*inliers, *coefficients);

    std::cout <<"inliers size " << inliers->indices.size () <<std::endl;
    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return (-1);
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
    //Buffer the coefficients
    cluster_coefficients[counter]=coefficients;

    counter++;
  }
  
  //Save the cluster coefficients
  saveCoefficients("filename.csv",cluster_coefficients);

  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Cluster viewer"));
  // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud); 
  // viewer->setBackgroundColor (0,0,0);
  // viewer->addPointCloud(colored_cloud,rgb,"sample cloud");
  // while (!viewer->wasStopped ())
  // {
  //   viewer->spinOnce(100);
  // }

  return (0);
}