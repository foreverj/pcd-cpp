#include "ConvexHullProcessor.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>
#include <vector>
#include <fstream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polyhedron_3<K> Polyhedron_3;
typedef K::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;

ConvexHullProcessor::ConvexHullProcessor()
{
    
}

int ConvexHullProcessor::getConvexHull()
{
    Polyhedron_3 poly;
    CGAL::convex_hull_3(this->points.begin(),this->points.end(),poly);
    
    return 0;
}