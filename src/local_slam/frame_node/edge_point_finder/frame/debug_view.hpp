#ifndef DEBUG_VIEW_HPP
#define DEBUG_VIEW_HPP

#include <opencv2/opencv.hpp>

#include "edge.h"

class DebugView {
public:
    DebugView(const cv::Mat& base_img);

    void drawEdgePoints(const std::vector<EdgePoint>& edge_points,
                        const cv::Scalar& color = cv::Scalar(0, 255, 0));

    cv::Mat getDebugImage() const;

private:
    cv::Mat base_img_;
};

#endif // DEBUG_VIEW_HPP
