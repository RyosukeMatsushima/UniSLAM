#ifndef FRAME_NODE_HPP
#define FRAME_NODE_HPP

#define HEALTHY_DISTANCE_RATIO 0.2f // compare with window_size
#define HEALTHY_ANGLE_RATIO 0.2f // compare with angle_resolution
#define KEYFRAME_EDGEPOINT_NUM_RATIO 0.8f // compare with fixed_edge_points.size() in previous frame

#define MIN_EDGEPOINT_NUM 1

#include <opencv2/opencv.hpp>

#include "frame.h"
#include "edge.h"
#include "edge_point_finder.hpp"


class FrameNode {
public:

    FrameNode(const cv::Mat& image,
              const int window_size,
              const float angle_resolution);

    bool matchEdge(const EdgePoint& edge_point,
                   EdgePoint& matched_edge_point);

    bool matchWith(const FrameNode& other_frame_node, bool& is_key_frame);

    std::vector<EdgePoint> findNewEdgePoints();

    void addFixedEdgePoint(const EdgePoint& edge_point);

    std::vector<EdgePoint> getFixedEdgePoints() const;

private:

    const int window_size;

    const float angle_resolution;

    std::vector<EdgePoint> fixed_edge_points;

    Frame frame_2d; //TODO: rename to Frame2D

    cv::Mat fixed_edge_distribution;
};
#endif // FRAME_NODE_HPP
