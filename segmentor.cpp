#include "segmentor.hpp"
#include "boundary.hpp"
#include <chrono>

float Segmentor::calculateAreaPolygon(const pcl::PointCloud<pcl::PointXYZ> &polygon)
{
  float area = 0.0;
  int num_points = polygon.size();
  int j =0;
  Eigen::Vector3f va,vb,res;
  res(0)=res(1)=res(2)=0.0f;
  for(int i = 0; i < num_points;++i)
  {
    j = (i+1)%num_points;
    va = polygon[i].getVector3fMap();
    vb = polygon[j].getVector3fMap();
    res += va.cross(vb);
  }
  area = res.norm();
  return area*0.5;
}

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
  auto start = std::chrono::steady_clock::now();
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
  auto end = std::chrono::steady_clock::now();
  auto diff = end - start;
  std::cout<<"Cluster Coefficients ... Done "<<std::chrono::duration<double,std::milli>(diff).count()<<" ms"<<std::endl;
  out.close();
}

void Segmentor::projectPointsAndSavePly(std::vector <pcl::ModelCoefficients::Ptr> cluster_coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::vector <pcl::PointIndices> clusters)
{
  int counter=0;
  const int numberOfClusters = cluster_coefficients.size();
  auto start = std::chrono::steady_clock::now();
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

    //Push Data to CGAL Boundary Paramatrization
    BoundaryProcessor bp(*cloud_projected,*cluster_coefficients[counter],this->options.DEBUG_MODE);
    bp.processData();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ccloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cluster_cloud,*ccloud);
    

    if(this->options.DISPLAY_MODE || this->options.SAVE_CLUSTERS_OF_INTEREST)
    {
      for(int i = 0; i < ccloud->points.size();i++)
      {
        ccloud->points[i].rgb = this->color_map[counter].rgb;
      }
    }
    // Check if the cluster satifies the minimum area requirement
    if (calculateAreaPolygon(bp.converted_points) >= this->options.MINIMUM_AREA){
      if(this->options.SAVE_EDGES_FOR_CLUSTERS_OF_INTEREST){
        bp.saveConvertedPoints("cluster"+std::to_string(counter)+"_edges.ply");
      }
      if(this->options.SAVE_PROJECTED_CLUSTERS_OF_INTEREST){
        pcl::io::savePLYFile("data/cluster"+std::to_string(counter)+"_projected.ply",*cloud_projected,false);
      }
      if(this->options.SAVE_CLUSTERS_OF_INTEREST){
        pcl::io::savePLYFile("data/cluster"+std::to_string(counter)+"_colored.ply",*ccloud,false);
      }
      pcl::PointCloud<pcl::PointXYZ>::Ptr egcloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(bp.converted_points,*egcloud);
      this->filtered_edges.push_back(egcloud);
      this->filtered_clouds.push_back(ccloud);
    }
    
    counter++;
  }
  auto end = std::chrono::steady_clock::now();
  auto diff = end - start;

  std::cout<<"Cluster Projection ... Done "<<std::chrono::duration<double,std::milli>(diff).count()<<" ms"<<std::endl;
}

Segmentor::Segmentor(std::string filename,SegmentorOptions options)
{

    auto start = std::chrono::steady_clock::now();
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
    auto end = std::chrono::steady_clock::now();

    auto diff = end - start;
    std::cout<<"Reading Point Cloud ... Done "<<std::chrono::duration<double,std::milli>(diff).count()<<" ms"<<std::endl;
    // Noise Removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfilter(true);
    rorfilter.setInputCloud(cloud);
    rorfilter.setRadiusSearch(options.NOISE_RADIUS);
    rorfilter.setMinNeighborsInRadius(options.NOISE_THRESHOLD);
    rorfilter.setNegative(false);
    rorfilter.filter(*filtered_cloud);
    
    this->cloud = filtered_cloud;
    this->options = options;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr proj_c (new pcl::PointCloud<pcl::PointXYZRGB>);

    this->projected_cloud = proj_c;
}

pcl::PointCloud <pcl::PointXYZRGB>::Ptr Segmentor::coloredCloud()
{
      pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = this->reg.getColoredCloud ();
      return colored_cloud;
}


int Segmentor::segment ()
{

  int counter = 0;
  auto start = std::chrono::steady_clock::now();
  std::cout<<"Segmentation ..."<<std::endl;
  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (this->cloud);
  normal_estimator.setKSearch (this->options.NORMALS_KSEARCH);
  normal_estimator.compute (*normals);

  // pcl::IndicesPtr indices (new std::vector <int>);
  // pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud (this->cloud);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (0.0, 1.0);
  // pass.filter (*indices);

  this->reg.setMinClusterSize (this->options.REGIONAL_GROWING_MIN_CLUSTER_SIZE);
  this->reg.setMaxClusterSize (this->options.REGIONAL_GROWING_MAX_CLUSTER_SIZE);
  this->reg.setSearchMethod (tree);
  this->reg.setNumberOfNeighbours (this->options.REGIONAL_GROWING_NUMBER_OF_NEIGHBOURS);
  this->reg.setInputCloud (cloud);
  // this->reg.setIndices(indices);
  this->reg.setInputNormals (normals);
  this->reg.setSmoothnessThreshold (this->options.REGIONAL_GROWING_SMOOTHNESS_THRESHOLD);
  this->reg.setCurvatureThreshold (this->options.REGIONAL_GROWING_CURVATURE_THRESHOLD);

  this->reg.extract (this->clusters);
  
  auto end = std::chrono::steady_clock::now();
  auto diff = end - start;
  std::cout<<"Segmentation ... Done "<<std::chrono::duration<double,std::milli>(diff).count()<<" ms"<<std::endl;
  
  //Fit plane equations to each cluster using RANSAC
  counter = 0;

  this->cluster_coefficients.resize(clusters.size());
  this->color_map.resize(clusters.size());

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = this->reg.getColoredCloud ();

  start = std::chrono::steady_clock::now();
  int numberOfClusters = clusters.size();
  while(counter<numberOfClusters)
  {
    
    printProgressBar("Plane Fitting",counter+1,numberOfClusters);

    //Get and store color map for the cluster
    this->color_map[counter].r = colored_cloud->points[clusters[counter].indices[0]].r;
    this->color_map[counter].g = colored_cloud->points[clusters[counter].indices[0]].g;
    this->color_map[counter].b = colored_cloud->points[clusters[counter].indices[0]].b;

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
    seg.setDistanceThreshold (this->options.SAC_DISTANCE_THRESHOLD);
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
  end = std::chrono::steady_clock::now();
  diff = end -start;
  std::cout<<"Plane Fitting ... Done "<<std::chrono::duration<double,std::milli>(diff).count()<<" ms"<<std::endl;
  
  projectPointsAndSavePly(this->cluster_coefficients,this->cloud,this->clusters);

  pcl::ModelCoefficients::Ptr yzPlaneCoefficients (new pcl::ModelCoefficients());

  // x=0
  yzPlaneCoefficients->values.resize(4);
  yzPlaneCoefficients->values[0]=1.0;
  yzPlaneCoefficients->values[1]=0;
  yzPlaneCoefficients->values[2]=0;
  yzPlaneCoefficients->values[3]=0;

  pcl::ModelCoefficients::Ptr xzPlaneCoefficients (new pcl::ModelCoefficients());

  // y=0
  xzPlaneCoefficients->values.resize(4);
  xzPlaneCoefficients->values[0]=0;
  xzPlaneCoefficients->values[1]=1;
  xzPlaneCoefficients->values[2]=0;
  xzPlaneCoefficients->values[3]=0;



  pcl::ProjectInliers<pcl::PointXYZRGB> yz_proj;
  pcl::ProjectInliers<pcl::PointXYZRGB> xz_proj;

  yz_proj.setModelType(pcl::SACMODEL_PLANE);
  yz_proj.setModelCoefficients(yzPlaneCoefficients);

  xz_proj.setModelType(pcl::SACMODEL_PLANE);
  xz_proj.setModelCoefficients(xzPlaneCoefficients);

  
  counter = 0;
  int numberOfFilteredClusters = this->filtered_clouds.size();

  this->_projected_clouds.resize(numberOfFilteredClusters);

  pcl::PointCloud<pcl::PointXYZRGB> pc;

  while(counter < numberOfFilteredClusters)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fc = filtered_clouds[counter];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr yz_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xz_pc (new pcl::PointCloud<pcl::PointXYZRGB>);

    yz_proj.setInputCloud (fc);
    yz_proj.filter (*yz_pc);

    xz_proj.setInputCloud(yz_pc);
    xz_proj.filter(*xz_pc);

    pc += *xz_pc;
    this->_projected_clouds.push_back(xz_pc);

    counter ++;
  }

  this->projected_cloud->points.resize(pc.points.size());

  for(int j=0;j<pc.points.size();j++)
  {
    this->projected_cloud->points[j] = pc.points[j];
  }





  
  return clusters.size();
  
}

