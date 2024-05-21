#include "frame_node.hpp"

FrameNode::FrameNode(const cv::Mat& image,
                     const int window_size,
                     const float angle_resolution)
                     : frame_2d(image, angle_resolution),
                       window_size(window_size),
                       angle_resolution(angle_resolution) {
    fixed_edge_distribution = cv::Mat::zeros(image.size() / window_size, CV_8UC1);
}

bool FrameNode::matchEdge(const EdgePoint& edge_point,
                          EdgePoint& matched_edge_point) {
    bool is_valid = false;

    // TODO: check usage of edge_point_finder. Is it necessary to create a new instance every time?
    EdgePointFinder edge_point_finder;
    matched_edge_point = edge_point_finder.find_key_edge_point(frame_2d,
                                                               edge_point.point,
                                                               edge_point.angle,
                                                               window_size,
                                                               is_valid);
    return is_valid;
}

void FrameNode::addFixedEdgePoint(const EdgePoint& edge_point) {
    fixed_edge_points.push_back(edge_point);

    fixed_edge_distribution.at<uchar>(edge_point.point / window_size) += 1;
}

std::vector<EdgePoint> FrameNode::findNewEdgePoints() {
    std::vector<EdgePoint> new_edge_points;

    for (int i = 1; i < fixed_edge_distribution.rows; i++) {
        for (int j = 1; j < fixed_edge_distribution.cols; j++) {
            if (fixed_edge_distribution.at<uchar>(i, j) >= 1)  continue;

            cv::Point2f point = cv::Point2f(j*window_size, i*window_size);

            for (float angle = 0.0f; angle < 2 * M_PI; angle += angle_resolution) {

                bool is_valid = false;
                EdgePoint edge_point = EdgePointFinder().find_key_edge_point(frame_2d,
                                                                             point,
                                                                             angle,
                                                                             window_size,
                                                                             is_valid);
                if (is_valid) new_edge_points.push_back(edge_point);
            }
        }
    }

    return new_edge_points;
}

std::vector<EdgePoint> FrameNode::getFixedEdgePoints() const {
    return fixed_edge_points;
}

bool FrameNode::isKeyFrame() const {
    return false;
}

