#include "segmentor.hpp"
#include "boundary.hpp"

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

void Segmentor::extractPointCloudsFromCoefficients(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ> &cluster_cloud,pcl::PointIndices::Ptr cluster)
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

void Segmentor::projectPointsAndSavePly(std::vector <pcl::ModelCoefficients::Ptr> cluster_coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::vector <pcl::PointIndices> clusters)
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
    bp.processData();
    bp.saveConvertedPoints("cluster"+std::to_string(counter)+"_edges.ply");

    counter++;
  }
  std::cout<<"Cluster Projection ... Done"<<std::endl;
}

Segmentor::Segmentor(std::string filename)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    std::string extension = filename.substr(filename.find_last_of(".") + 1);

    if(extension == "pcd") {
      if ( pcl::io::loadPCDFile <pcl::PointXYZ> (filename, *cloud) == -1)
      {
          throw std::invalid_argument( "Cloud reading failed" );
      }
    } else if(extension == "ply") {
      if ( pcl::io::loadPLYFile <pcl::PointXYZ> (filename, *cloud) == -1)
      {
          throw std::invalid_argument( "Cloud reading failed" );
      }
    } else{
      throw std::invalid_argument( "File format not supported" );
    }

    // Noise Removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfilter(true);
    rorfilter.setInputCloud(cloud);
    rorfilter.setRadiusSearch(0.1);
    rorfilter.setMinNeighborsInRadius(3);
    rorfilter.setNegative(false);
    rorfilter.filter(*filtered_cloud);
    
    this->cloud = filtered_cloud;
}

pcl::PointCloud <pcl::PointXYZRGB>::Ptr Segmentor::coloredCloud()
{
      pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = this->reg.getColoredCloud ();
      return colored_cloud;
}


int Segmentor::segment ()
{

  int counter = 0;
  

  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (this->cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (this->cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  this->reg.setMinClusterSize (50);
  this->reg.setMaxClusterSize (1000000);
  this->reg.setSearchMethod (tree);
  this->reg.setNumberOfNeighbours (30);
  this->reg.setInputCloud (cloud);
  this->reg.setInputNormals (normals);
  this->reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  this->reg.setCurvatureThreshold (1.0);

  this->reg.extract (this->clusters);

  //Fit plane equations to each cluster using RANSAC
  counter = 0;

  this->cluster_coefficients.resize(clusters.size());

  int numberOfClusters = clusters.size();
  while(counter<numberOfClusters)
  {
    
    printProgressBar("Plane Fitting",counter+1,numberOfClusters);

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

    //Buffer the coefficients
    this->cluster_coefficients[counter]=coefficients;

    counter++;
  }
  std::cout<<"Plane Fitting ... Done"<<std::endl;
  
  projectPointsAndSavePly(this->cluster_coefficients,this->cloud,this->clusters);
  
  return clusters.size();
  
}

