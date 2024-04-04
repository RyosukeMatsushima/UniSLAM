#ifndef DEBUG_VIEW_HPP
#define DEBUG_VIEW_HPP

#include <opencv2/opencv.hpp>

#include "edge.h"

class DebugView {
public:
    DebugView(const cv::Mat& base_img);

    void drawEdgePoints(const std::vector<EdgePoint>& edge_points);

    cv::Mat getDebugImage() const;

private:
    cv::Mat base_img_;
};

#endif // DEBUG_VIEW_HPP
