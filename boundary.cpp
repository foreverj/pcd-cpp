#include "boundary.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>

void BoundaryProcessor::processEdges()
{
    std::vector<Segment> segments = _converted_edges;
    int counter=0;
    const int numberOfSegments = segments.size();
    //std::vector<pcl::PointXYZ> converted_points (new pcl::PointXYZ);
    this->converted_points.resize(2*numberOfSegments);
    while(counter<numberOfSegments)
  {
    
    for(int j=0;j<2;j++)
    {
        pcl::PointXYZ vertex;
        vertex.x = segments[counter].vertex(j).x();
        vertex.y = segments[counter].vertex(j).y();
        vertex.z = 0.0;
        pcl::PointXYZ cp = twoDtoThreeD(vertex);
        this->converted_points.push_back(cp);
    }

    counter++;
  }
}
void BoundaryProcessor::saveConvertedPoints(std::string filename)
{
    pcl::io::savePLYFile("data/"+filename,this->converted_points,false);
}


void BoundaryProcessor::saveEdges(std::string filename)
{
  std::string header="edge number,x1,y1,x2,y2\n";
  std::ofstream out(filename);
  out << header;
  std::vector<Segment> segments = _converted_edges;
  int counter=0;
  const int numberOfSegments = segments.size();
  while(counter<numberOfSegments)
  {
    out<<""<<counter<<","
                <<segments[counter].vertex(0).x()<<","
                <<segments[counter].vertex(0).y()<<","
                <<segments[counter].vertex(1).x()<<","
                <<segments[counter].vertex(1).y()<<"\n";
    counter++;
  }
  out.close();
}

template <class OutputIterator>
void BoundaryProcessor::alpha_edges(const Alpha_shape_2& A, OutputIterator out)
{
    Alpha_shape_edges_iterator it = A.alpha_shape_edges_begin(),
                               end = A.alpha_shape_edges_end();
    for(;it!=end;++it)
    {
        *out++ = A.segment(*it);
    }
}


template <class OutputIterator>
bool BoundaryProcessor::file_input(OutputIterator out)
{
    std::ifstream is("./data/fin",std::ios::in);

    if(is.fail())
    {
        std::cerr<<"Unable to open file for input"<<std::endl;
        return false;
    }

    int n;
    is >> n;
    std::cout<<"Reading "<<n<<" points from file"<<std::endl;
    CGAL::cpp11::copy_n(std::istream_iterator<Point>(is),n,out);
    return true;
}


pcl::PointXYZ BoundaryProcessor::twoDtoThreeD(pcl::PointXYZ original)
{
    
    // Rotation Matrix
    pcl::PointXYZ result;
    result.x = this->_x_prime.x*original.x + this->_y_prime.x*original.y + this->_z_prime.x*original.z + this->_centroid.x;
    result.y = this->_x_prime.y*original.x + this->_y_prime.y*original.y + this->_z_prime.y*original.z + this->_centroid.y;
    result.z = this->_x_prime.z*original.x + this->_y_prime.z*original.y + this->_z_prime.z*original.z + this->_centroid.z;

    return result;
}

pcl::PointXYZ BoundaryProcessor::threeDtoTwoD(pcl::PointXYZ original)
{
    pcl::PointXYZ pointMoveToOrigin;
    pointMoveToOrigin.x = original.x - this->_centroid.x;
    pointMoveToOrigin.y = original.y - this->_centroid.y;
    pointMoveToOrigin.z = original.z - this->_centroid.z;
    // Rotation Matrix
    pcl::PointXYZ result;
    result.x = _x_prime.x*pointMoveToOrigin.x + _x_prime.y*pointMoveToOrigin.y + _x_prime.z*pointMoveToOrigin.z;
    result.y = _y_prime.x*pointMoveToOrigin.x + _y_prime.y*pointMoveToOrigin.y + _y_prime.z*pointMoveToOrigin.z;
    result.z = _z_prime.x*pointMoveToOrigin.x + _z_prime.y*pointMoveToOrigin.y + _z_prime.z*pointMoveToOrigin.z;

    return result;
}

template <class OutputIterator>
bool BoundaryProcessor::pointCloudInput(pcl::PointCloud<pcl::PointXYZ> cloud_projection, pcl::ModelCoefficients coeff,OutputIterator out)
{
    pcl::PointCloud<pcl::PointXYZ>::iterator b1;
    
    int n = cloud_projection.points.size();
    std::ostringstream out_string;
    int i = 0;
    for(b1=cloud_projection.points.begin();b1<cloud_projection.points.end();b1++)
    {
        pcl::PointXYZ original;
        original.x = b1->x;
        original.y = b1->y;
        original.z = b1->z;
        pcl::PointXYZ transformed_point;
        transformed_point = threeDtoTwoD(original);
        if(this->debug_mode)
        {
            if (i<10){
                std::cout<<transformed_point.x<<" "<<transformed_point.y<<" "<<transformed_point.z<<std::endl;
                i++;
            }
        }
        
        out_string<<std::to_string(transformed_point.x) <<" "<<std::to_string(transformed_point.y)<<std::endl;
    }

    std::istringstream is(out_string.str());
    CGAL::cpp11::copy_n(std::istream_iterator<Point>(is),n,out);
    return true;
}

void BoundaryProcessor::alpha_compute_output(std::list<Point> points)
{
    Alpha_shape_2 A(points.begin(),points.end(),FT(10000),Alpha_shape_2::GENERAL);

    Alpha_shape_2::Alpha_iterator opt = A.find_optimal_alpha(1);
    A.set_alpha(*opt);

    std::vector<Segment> segments;
    alpha_edges(A,std::back_inserter(segments));

    _converted_edges = segments;
    processEdges();

}

void BoundaryProcessor::alpha_compute_output(std::list<Point> points, std::string filename)
{
    Alpha_shape_2 A(points.begin(),points.end(),FT(10000),Alpha_shape_2::GENERAL);

    Alpha_shape_2::Alpha_iterator opt = A.find_optimal_alpha(1);
    A.set_alpha(*opt);

    std::vector<Segment> segments;
    alpha_edges(A,std::back_inserter(segments));
    _converted_edges = segments;
    processEdges();
    saveEdges(filename);
}

void BoundaryProcessor::processData()
{
    std::list<Point> points;
    if(! pointCloudInput(_cloud_projection,_coeff,std::back_inserter(points)))
    {
        std::cerr<<"Unable to read points"<<std::endl;
    } 

    alpha_compute_output(points);
}

void BoundaryProcessor::processData(std::string filename)
{
    std::list<Point> points;
    if(! pointCloudInput(_cloud_projection,_coeff,std::back_inserter(points)))
    {
        std::cerr<<"Unable to read points"<<std::endl;
    } 

    alpha_compute_output(points,filename);
}

BoundaryProcessor::BoundaryProcessor(pcl::PointCloud<pcl::PointXYZ> cloud_projection,pcl::ModelCoefficients coeff,bool debug_mode)
{
    _cloud_projection = cloud_projection;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cloud_projection,centroid);
    _centroid.x = centroid[0];
    _centroid.y = centroid[1];
    _centroid.z = centroid[2];
    _coeff = coeff;
    
    //define normal vector
    pcl::PointXYZ normal;
    normal.x = coeff.values[0];
    normal.y = coeff.values[1];
    normal.z = coeff.values[2];

    //normalize vector and assign the normalized vector as Z' axis
    pcl::PointXYZ z_prime;
    double length = sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
    z_prime.x = normal.x/length;
    z_prime.y = normal.y/length;
    z_prime.z = normal.z/length;

    //assume X' axis always has y=0
    pcl::PointXYZ x_prime;
    x_prime.z = sqrt(1.0/((z_prime.z)/(z_prime.x)*(z_prime.z)/(z_prime.x)+1.0));
    x_prime.y = 0;
    x_prime.x = -z_prime.z/z_prime.x*x_prime.z;

    // Y' axis = Z' cross X'
    pcl::PointXYZ y_prime;
    y_prime.x = z_prime.y*x_prime.z-z_prime.z*x_prime.y;
    y_prime.y = z_prime.z*x_prime.x-z_prime.x*x_prime.z;
    y_prime.z = z_prime.x*x_prime.y-z_prime.y*x_prime.x;

    _x_prime = x_prime;
    _y_prime = y_prime;
    _z_prime = z_prime;

    this->debug_mode = debug_mode;
}