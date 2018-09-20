#include "boundary.h"

void BoundaryProcessor::saveEdges(std::string filename,std::vector<Segment> segments)
{
  std::string header="edge number,x1,y1,x2,y2\n";
  std::ofstream out(filename);
  out << header;

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

void BoundaryProcessor::saveEdges(std::string filename,std::vector<Point> vertices)
{
  std::string header="x,y";
  std::ofstream out(filename);
  out << header;

  int counter=0;
  const int numberOfSegments = vertices.size();
  while(counter<numberOfSegments)
  {
    out<<""<<vertices[counter].x()<<","
        <<vertices[counter].y()<<"\n";
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


pcl::PointXYZ BoundaryProcessor::threeDtoTwoD(pcl::PointXYZ original)
{
    
    // Rotation Matrix
    pcl::PointXYZ result;
    result.x = this->_x_prime.x*original.x + this->_y_prime.x*original.y + this->_z_prime.x*original.z;
    result.y = this->_x_prime.y*original.x + this->_y_prime.y*original.y + this->_z_prime.y*original.z;
    result.z = this->_x_prime.z*original.x + this->_y_prime.z*original.y + this->_z_prime.z*original.z;

    return result;
}

pcl::PointXYZ BoundaryProcessor::twoDtoThreeD(pcl::PointXYZ original)
{

    // Rotation Matrix
    pcl::PointXYZ result;
    result.x = _x_prime.x*original.x + _x_prime.y*original.y + _x_prime.z*original.z;
    result.y = _y_prime.x*original.x + _y_prime.y*original.y + _y_prime.z*original.z;
    result.z = _z_prime.x*original.x + _z_prime.y*original.y + _z_prime.z*original.z;

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
        if (i<10){
            std::cout<<transformed_point.x<<" "<<transformed_point.y<<" "<<transformed_point.z<<std::endl;
            i++;
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

}

void BoundaryProcessor::alpha_compute_output(std::list<Point> points, std::string filename)
{
    Alpha_shape_2 A(points.begin(),points.end(),FT(10000),Alpha_shape_2::GENERAL);

    Alpha_shape_2::Alpha_iterator opt = A.find_optimal_alpha(1);
    A.set_alpha(*opt);

    std::vector<Segment> segments;
    alpha_edges(A,std::back_inserter(segments));

    saveEdges(filename,segments);
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

BoundaryProcessor::BoundaryProcessor(pcl::PointCloud<pcl::PointXYZ> cloud_projection,pcl::ModelCoefficients coeff)
{
    _cloud_projection = cloud_projection;
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
    x_prime.z = 1.0/((z_prime.z)/(z_prime.x)*(z_prime.z)/(z_prime.x)+1.0);
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
}