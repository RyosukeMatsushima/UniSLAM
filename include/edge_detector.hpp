#ifndef EDGE_DETECTOR_HPP
#define EDGE_DETECTOR_HPP

#include <opencv2/opencv.hpp>

#include "edge.h"

#define BLOCK_SIZE 6
#define TRIAL_NUM 10
#define MIN_EDGE_SIZE 50
#define MAX_FAILED_TRIAL_NUM 5
#define EDGE_CLEARANCE 10   // pixels

class EdgeDetector {
public:
    EdgeDetector(const cv::Mat& laplacian_img,
                 const cv::Mat& gradient_x,
                 const cv::Mat& gradient_y);

    std::vector<EdgePoint> getKeyEdgePoints();

private:
    cv::Mat edge_intensity;
    cv::Mat gradient_x;
    cv::Mat gradient_y;

    std::vector<Edge> getEdges(const block_index_x,
                               const block_index_y);

    Edge traceEdge(const cv::Point& start_point)

    cv::Vec2f getGradient(const cv::Point& point);

    bool getNextEdgePoint(const cv::Point& current_point,
                          const float& angle,
                          const bool is_forward,
                          cv::Point& next_point);
}

#endif // EDGE_DETECTOR_HPP
