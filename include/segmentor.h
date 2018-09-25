#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

class Segmentor{
    public:
        Segmentor(std::string filename);
        int segment ();
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr coloredCloud();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        std::vector <pcl::ModelCoefficients::Ptr> cluster_coefficients;
        std::vector <pcl::PointIndices> clusters;
};
