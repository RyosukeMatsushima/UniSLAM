#ifndef EDGE_H
#define EDGE_H

#include <opencv2/opencv.hpp>

#include "edge_point.hpp"



class Edge {
public:
    Edge();

    bool addPoint(const cv::Point2f& point,
                  const cv::Vec2f& gradient);

    std::vector<EdgePoint> getEdgePoints() const;

    bool joinEdge(Edge& joining_edge);

    std::vector<EdgePoint> getEdgePoints();

    int size() const;

private:
    std::vector<EdgePoint> edge_points_;
};

#endif // EDGE_H
