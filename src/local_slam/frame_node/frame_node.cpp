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
    matched_edge_point.id = edge_point.id;
    return is_valid;
}

bool FrameNode::matchWith(const FrameNode& other_frame_node, bool& is_key_frame) {

    int healthy_points_num = 0;
    int matched_points_num = 0;

    float distance_threshold = window_size * HEALTHY_DISTANCE_RATIO;
    float angle_threshold = angle_resolution * HEALTHY_ANGLE_RATIO;

    for (const EdgePoint& fixed_edge_point : other_frame_node.getFixedEdgePoints()) {
        EdgePoint matched_edge_point(cv::Point2f(0, 0), cv::Vec2f(0, 0));
        bool result = matchEdge(fixed_edge_point, matched_edge_point);

        if (!result) continue;

        addFixedEdgePoint(matched_edge_point);
        matched_points_num++;

        if (matched_edge_point.distanceTo(fixed_edge_point) > distance_threshold) continue;

        if (matched_edge_point.angleTo(fixed_edge_point) > angle_threshold) continue;

        healthy_points_num++;
    }

    int keyframe_edgepoint_num_threshold = other_frame_node.getFixedEdgePoints().size() * KEYFRAME_EDGEPOINT_NUM_RATIO;

    if (matched_points_num <= MIN_EDGEPOINT_NUM) { //TODO: reconsider this
        is_key_frame = false;
        return false;
    }

    is_key_frame = healthy_points_num <= keyframe_edgepoint_num_threshold;

    return true;
}

void FrameNode::addFixedEdgePoint(const EdgePoint& edge_point) {
    fixed_edge_points.push_back(edge_point);

    fixed_edge_distribution.at<uchar>(edge_point.point / window_size) += 1;
}

std::vector<EdgePoint> FrameNode::findNewEdgePoints() const {
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

void FrameNode::shuffleFixedEdgePoints() {
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(fixed_edge_points.begin(), fixed_edge_points.end(), g);
}

FrameNode& FrameNode::operator=(const FrameNode& other_frame_node) {

    // check window_size and angle_resolution
    if (window_size != other_frame_node.window_size) {
        throw std::invalid_argument("window_size is different");
    }
    if (angle_resolution != other_frame_node.angle_resolution) {
        throw std::invalid_argument("angle_resolution is different");
    }

    frame_2d = other_frame_node.frame_2d;
    fixed_edge_points = other_frame_node.fixed_edge_points;
    fixed_edge_distribution = other_frame_node.fixed_edge_distribution.clone();
    return *this;
}

