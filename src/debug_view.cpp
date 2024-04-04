
#include "debug_view.hpp"

DebugView::DebugView(const cv::Mat& base_img) {
    base_img_ = base_img.clone();
    cv::cvtColor(base_img_, base_img_, cv::COLOR_GRAY2BGR);
}

void DebugView::drawEdgePoints(const std::vector<EdgePoint>& edge_points) {

    // draw edge points and normalized gradients as arrows

    for (const auto& edge_point : edge_points) {
        cv::circle(base_img_, edge_point.point, 2, cv::Scalar(0, 0, 255), -1);

        cv::Point2f gradient = edge_point.gradient;
        gradient = gradient / cv::norm(gradient);
        cv::Point2f gradient_end = edge_point.point + gradient * 15;

        cv::arrowedLine(base_img_, edge_point.point, gradient_end, cv::Scalar(0, 255, 0), 1);

    }
}

cv::Mat DebugView::getDebugImage() const {
    return base_img_;
}
