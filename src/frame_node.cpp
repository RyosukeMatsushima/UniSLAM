#include "frame_node.hpp"

FrameNode::FrameNode(const cv::Mat& image) : frame_2d(image, ANGLE_RESOLUTION) {
    fixed_edge_distribution = cv::Mat::zeros(image.size() / WINDOW_SIZE, CV_8UC1);
}

bool FrameNode::matchEdge(const EdgePoint& edge_point,
                          EdgePoint& matched_edge_point) {
    bool is_valid = false;

    // TODO: check usage of edge_point_finder. Is it necessary to create a new instance every time?
    EdgePointFinder edge_point_finder;
    matched_edge_point = edge_point_finder.find_key_edge_point(frame_2d,
                                                               edge_point.point,
                                                               edge_point.angle,
                                                               WINDOW_SIZE,
                                                               is_valid);
    return is_valid;
}

void FrameNode::addFixedEdgePoint(const EdgePoint& edge_point) {
    fixed_edge_points.push_back(edge_point);

    fixed_edge_distribution.at<uchar>(edge_point.point / WINDOW_SIZE) += 1;
}

std::vector<EdgePoint> FrameNode::findNewEdgePoints() {
    std::vector<EdgePoint> new_edge_points;

    for (int i = 0 + WINDOW_SIZE / 2; i < fixed_edge_distribution.rows; i+=WINDOW_SIZE) {
        for (int j = 0 + WINDOW_SIZE / 2; j < fixed_edge_distribution.cols; j+=WINDOW_SIZE){
            if (fixed_edge_distribution.at<uchar>(i, j) >= 1)  continue;

            cv::Point2f point = cv::Point2f(j, i);
            bool is_valid = false;

            for (float angle = 0.0f; angle < 2 * M_PI; angle += ANGLE_RESOLUTION) {
                EdgePoint edge_point = EdgePointFinder().find_key_edge_point(frame_2d,
                                                                             point,
                                                                             angle,
                                                                             WINDOW_SIZE,
                                                                             is_valid);
                if (is_valid) {
                    new_edge_points.push_back(edge_point);
                    break;
                }
            }
        }
    }

    return new_edge_points;
}

