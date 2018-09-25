#include <fstream>
#include <iostream>
#include <vector>

#include "boundary.h"

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

#ifdef DEBUG

#include <pcl/visualization/pcl_visualizer.h>

#endif

void printProgressBar(std::string label, int step, int total){

  //Progress width
  const int pwidth = 72;
  int width = pwidth - label.size();
  int pos = (step*width)/total;

  int percent = (step*100)/total;
  std::string bar;

  for(int i = 0; i < 50; i++){
    if( i < (percent/2)){
      bar.replace(i,1,"=");
    }else if( i == (percent/2)){
      bar.replace(i,1,">");
    }else{
      bar.replace(i,1," ");
    }
  }

  std::cout<<"\r" <<label <<"[" << bar << "] ";
  std::cout.width( 3 );
  std::cout<< percent << "%     " << std::flush;
  
  if(percent==100){
    std::cout<<std::endl;
  }
}

void extractPointCloudsFromCoefficients(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ> &cluster_cloud,pcl::PointIndices::Ptr cluster)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud (cloud);
    extract.setIndices (cluster);
    extract.setNegative (false);
    extract.filter (cluster_cloud);
}

void saveCoefficients(std::string filename,std::vector <pcl::ModelCoefficients::Ptr> cluster_coefficients)
{
  std::string header="cluster number,coefficient 0,coefficient 1,coefficient 2,coefficient 3\n";
  std::ofstream out(filename);
  out << header;

  int counter=0;
  const int numberOfClusters = cluster_coefficients.size();
  while(counter<numberOfClusters)
  {
    printProgressBar("Cluster Coefficients",counter+1,numberOfClusters);
    out<<""<<counter<<","
                <<cluster_coefficients[counter]->values[0]<<","
                <<cluster_coefficients[counter]->values[1]<<","
                <<cluster_coefficients[counter]->values[2]<<","
                <<cluster_coefficients[counter]->values[3]<<"\n";
    counter++;
  }
  std::cout<<"Cluster Coefficients ... Done"<<std::endl;
  out.close();
}

void projectPointsAndSavePly(std::vector <pcl::ModelCoefficients::Ptr> cluster_coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::vector <pcl::PointIndices> clusters)
{
  int counter=0;
  const int numberOfClusters = cluster_coefficients.size();
  while(counter<numberOfClusters)
  {
    printProgressBar("Cluster Projection",counter+1,numberOfClusters);
    //Cluster point cloud extraction variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr cluster (new pcl::PointIndices);

    int index_counter = 0;
    while(index_counter<clusters[counter].indices.size())
    {
      cluster->indices.resize(clusters[counter].indices.size());
      cluster->indices[index_counter]=clusters[counter].indices[index_counter];
      index_counter++;
    }

    extractPointCloudsFromCoefficients(cloud,*cluster_cloud,cluster);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cluster_cloud);
    proj.setModelCoefficients (cluster_coefficients[counter]);
    proj.filter (*cloud_projected);
    pcl::io::savePLYFile("data/cluster"+std::to_string(counter)+"_projected.ply",*cloud_projected,false);

    //Push Data to CGAL Boundary Paramatrization
    BoundaryProcessor bp(*cloud_projected,*cluster_coefficients[counter]);
    if(counter == 0)
    {
      bp.processData("cluster0_edges.csv");
      bp.saveConvertedPoints("cluster0_processed.ply");

    }else{
      bp.processData();
    }
    

    counter++;
  }
  std::cout<<"Cluster Projection ... Done"<<std::endl;
}

int main (int argc, char** argv)
{
  //Test Static Library Linkage
  // BoundaryProcessor::processData();

  int counter = 0;
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
  
  #ifdef DEBUG
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
  std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;
  while (counter < clusters[0].indices.size ())
  {
    std::cout << clusters[0].indices[counter] << ", ";
    counter++;
    if (counter % 10 == 0)
      std::cout << std::endl;
  }
  std::cout << std::endl;
  #endif

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::io::savePCDFile("colored_cloud.pcd",*colored_cloud);
  pcl::io::savePLYFile("colored_cloud.ply",*colored_cloud,false);

  //Fit plane equations to each cluster using RANSAC
  counter = 0;

  std::vector <pcl::ModelCoefficients::Ptr> cluster_coefficients;
  cluster_coefficients.resize(clusters.size());

  int numberOfClusters = clusters.size();
  while(counter<numberOfClusters)
  {
    
    #ifdef DEBUG
    std::cout <<"Start processing cluster " << counter <<std::endl;
    #else
    printProgressBar("Plane Fitting",counter+1,numberOfClusters);
    #endif

    //Cluster point cloud extraction variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr cluster (new pcl::PointIndices);

    int index_counter = 0;
    while(index_counter<clusters[counter].indices.size())
    {
      cluster->indices.resize(clusters[counter].indices.size());
      cluster->indices[index_counter]=clusters[counter].indices[index_counter];
      index_counter++;
    }

    extractPointCloudsFromCoefficients(cloud,*cluster_cloud,cluster);
    

    #ifdef DEBUG
    pcl::io::savePLYFile("data/cluster"+std::to_string(counter)+".ply",*cluster_cloud,false);
    #endif

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

    
    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return (-1);
    }

    #ifdef DEBUG 
    std::cout <<"inliers size " << inliers->indices.size () <<std::endl;
    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
    
    #endif
    //Buffer the coefficients
    cluster_coefficients[counter]=coefficients;

    counter++;
  }
  std::cout<<"Plane Fitting ... Done"<<std::endl;
  
  //Save the cluster coefficients
  saveCoefficients("coefficients.csv",cluster_coefficients);
  projectPointsAndSavePly(cluster_coefficients,cloud,clusters);

 

  //Launch the viewer to show segmentation results if in DEBUG mode
  #ifdef DEBUG
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Cluster viewer"));
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud); 
  viewer->setBackgroundColor (0,0,0);
  viewer->addPointCloud(colored_cloud,rgb,"sample cloud");
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce(100);
  }

  #endif

  return (0);
}