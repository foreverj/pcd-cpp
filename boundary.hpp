#ifndef _BOUNDARYPROCESSOR_H
#define _BOUNDARYPROCESSOR_H

#include <math.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>

#include <CGAL/algorithm.h>
#include <CGAL/assertions.h>

#include <fstream>
#include <iostream>
#include <list>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::FT                                               FT;
typedef K::Point_2                                          Point;
typedef K::Segment_2                                        Segment;

typedef CGAL::Alpha_shape_vertex_base_2<K>                  Vb;
typedef CGAL::Alpha_shape_face_base_2<K>                    Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>         Tds;
typedef CGAL::Delaunay_triangulation_2<K,Tds>               Triangulation_2;
typedef CGAL::Alpha_shape_2<Triangulation_2>                Alpha_shape_2;

typedef Alpha_shape_2::Alpha_shape_edges_iterator           Alpha_shape_edges_iterator;
typedef Alpha_shape_2::Alpha_shape_vertices_iterator        Alpha_shape_vertices_iterator;

class BoundaryProcessor
{
    public:
        BoundaryProcessor(pcl::PointCloud<pcl::PointXYZ> cloud_projection,pcl::ModelCoefficients coeff,bool debug_mode=false);
        void processData();
        void processData(std::string filename);
        void saveConvertedPoints(std::string filename);
        bool debug_mode;
        pcl::PointCloud<pcl::PointXYZ> converted_points;

    private:
        pcl::PointXYZ _x_prime;
        pcl::PointXYZ _y_prime;
        pcl::PointXYZ _z_prime;
        pcl::PointXYZ _centroid;
        pcl::PointCloud<pcl::PointXYZ> _cloud_projection;
        pcl::ModelCoefficients _coeff;
        pcl::PointXYZ threeDtoTwoD(pcl::PointXYZ original);
        pcl::PointXYZ twoDtoThreeD(pcl::PointXYZ original);
        std::vector<Segment> _converted_edges;
        template <class OutputIterator> bool pointCloudInput(pcl::PointCloud<pcl::PointXYZ> cloud_projection, pcl::ModelCoefficients coeff,OutputIterator out);
        void saveEdges(std::string filename);
        template <class OutputIterator> void alpha_edges(const Alpha_shape_2& A, OutputIterator out);
        template <class OutputIterator> bool file_input(OutputIterator out);
        void alpha_compute_output(std::list<Point> points);
        void alpha_compute_output(std::list<Point> points, std::string filename);
        void processEdges();
};

#endif