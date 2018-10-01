#ifndef _CONVEX_HULL_PROCESSOR_H
#define _CONVEX_HULL_PROCESSOR_H

class ConvexHullProcessor
{
    public:
        ConvexHullProcessor();
        int getConvexHull();
        std::vector<Point_3> points;
};

#endif