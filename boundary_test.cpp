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

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  // Fill in the cloud data
  cloud.width    = 4;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  cloud.points[0].x = 0.0;
  cloud.points[0].y = 0.0;
  cloud.points[0].z = 0.0;

  cloud.points[1].x = 0.0;
  cloud.points[1].y = 1.0;
  cloud.points[1].z = 0.0;

  cloud.points[2].x = 1.0;
  cloud.points[2].y = 1.0;
  cloud.points[2].z = 0.0;

  cloud.points[3].x = 1.0;
  cloud.points[3].y = 0.0;
  cloud.points[3].z = 1.0;

  pcl::ModelCoefficients coeff = new pcl::ModelCoefficients();
  coeff.values.resize(4);
  coeff.values[0] = 0.0;
  coeff.values[1] = 0.0;
  coeff.values[2] = 1.0;
  coeff.values[3] = 0.0;
 
  BoundaryProcessor bp(cloud,coeff);

  bp.processData("boundary_test.csv");

  return(0);
}