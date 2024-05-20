#ifndef FRAME_NODE_HPP
#define FRAME_NODE_HPP

#define ANGLE_RESOLUTION 0.2f
#define WINDOW_SIZE 50

#include <opencv2/opencv.hpp>

#include "frame.h"
#include "edge.h"
#include "edge_point_finder.hpp"


class FrameNode {
public:

    FrameNode(const cv::Mat& image);

    bool matchEdge(const EdgePoint& edge_point,
                   EdgePoint& matched_edge_point);
    
    void addFixedEdgePoint(const EdgePoint& edge_point);

    std::vector<EdgePoint> findNewEdgePoints();

    std::vector<EdgePoint> getFixedEdgePoints() const;

    bool isKeyFrame() const;

private:

    std::vector<EdgePoint> fixed_edge_points;

    Frame frame_2d; //TODO: rename to Frame2D

    cv::Mat fixed_edge_distribution;
};
#endif // FRAME_NODE_HPP
