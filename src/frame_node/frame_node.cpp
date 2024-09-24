#include "frame_node.hpp"

FrameNode::FrameNode(const cv::Mat& image,
                     const int window_size,
                     const float angle_resolution)
                     : frame_2d(image, angle_resolution),
                       window_size(window_size),
                       angle_resolution(angle_resolution),
                       fixed_edge_distribution(image.size(), cv::Size(window_size, window_size)) {
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

//    if (matched_points_num <= MIN_EDGEPOINT_NUM) { //TODO: reconsider this
//        is_key_frame = false;
//        std::cout << "Not enough matched points" << std::endl;
//        return false;
//    }

    is_key_frame = healthy_points_num <= keyframe_edgepoint_num_threshold;

    return true;
}

void FrameNode::addFixedEdgePoint(const EdgePoint& edge_point) {
    // check id is assigned
    if (edge_point.id == -1) {
        throw std::invalid_argument("id is not assigned");
    }

    fixed_edge_points.push_back(edge_point);
    fixed_edge_point_ids.push_back(edge_point.id);

    fixed_edge_distribution.add_edge_point(edge_point.point);
}

void FrameNode::removeFixedEdgePoint(const int edge_point_id) {
    int index = getEdgePointIndex(edge_point_id);

    fixed_edge_distribution.remove_edge_point(fixed_edge_points[index].point);

    fixed_edge_points.erase(fixed_edge_points.begin() + index);
    fixed_edge_point_ids.erase(fixed_edge_point_ids.begin() + index);
}

std::vector<EdgePoint> FrameNode::findNewEdgePoints() const {
    std::vector<EdgePoint> new_edge_points;

    std::vector<cv::Point> empty_windows = fixed_edge_distribution.get_empty_windows();

    for (const cv::Point& point : empty_windows) {
        for (float angle = 0.0f; angle < 2 * M_PI - angle_resolution; angle += angle_resolution) {
            EdgePointFinder edge_point_finder;
            bool is_valid = false;
            EdgePoint edge_point = edge_point_finder.find_key_edge_point(frame_2d,
                                                                        point,
                                                                        angle,
                                                                        window_size,
                                                                        is_valid);

            if (is_valid) {
                new_edge_points.push_back(edge_point);
            }
        }
    }

    return new_edge_points;
}

EdgePoint FrameNode::getFixedEdgePoint(const int edge_point_id) const {
    int index = getEdgePointIndex(edge_point_id);
    return fixed_edge_points[index];
}

std::vector<EdgePoint> FrameNode::getFixedEdgePoints() const {
    return fixed_edge_points;
}

void FrameNode::shuffleFixedEdgePoints() {
    std::vector<size_t> indices(fixed_edge_points.size());
    for (size_t i = 0; i < indices.size(); i++) {
        indices[i] = i;
    }

    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(indices.begin(), indices.end(), g);

    std::vector<EdgePoint> shuffled_fixed_edge_points;
    std::vector<int> shuffled_fixed_edge_point_ids;

    for (size_t i = 0; i < indices.size(); i++) {
        shuffled_fixed_edge_points.push_back(fixed_edge_points[indices[i]]);
        shuffled_fixed_edge_point_ids.push_back(fixed_edge_point_ids[indices[i]]);
    }

    fixed_edge_points = std::move(shuffled_fixed_edge_points);
    fixed_edge_point_ids = std::move(shuffled_fixed_edge_point_ids);
}

void FrameNode::clearFixedEdgePoints() {
    fixed_edge_points.clear();
    fixed_edge_point_ids.clear();
    fixed_edge_distribution.clear();
}

void FrameNode::moveFixedEdgePointToBack(const int edge_point_id) {
    int index = getEdgePointIndex(edge_point_id);

    fixed_edge_points.push_back(fixed_edge_points[index]);
    fixed_edge_point_ids.push_back(fixed_edge_point_ids[index]);

    fixed_edge_points.erase(fixed_edge_points.begin() + index);
    fixed_edge_point_ids.erase(fixed_edge_point_ids.begin() + index);
}

FrameNode& FrameNode::operator=(const FrameNode& other_frame_node) {

    // check window_size and angle_resolution
    if (window_size != other_frame_node.window_size) {
        throw std::invalid_argument("window_size is different");
    }
    if (angle_resolution != other_frame_node.angle_resolution) {
        throw std::invalid_argument("angle_resolution is different");
    }

    // TODO: it is possible to miss some variables
    frame_2d = other_frame_node.frame_2d;
    fixed_edge_points = std::move(other_frame_node.fixed_edge_points);
    fixed_edge_point_ids = std::move(other_frame_node.fixed_edge_point_ids);
    fixed_edge_distribution = other_frame_node.fixed_edge_distribution;
    return *this;
}

cv::Mat FrameNode::getImg() const {
    return frame_2d.getGrayImage();
}

int FrameNode::getEdgePointIndex(const int edge_point_id) const {

    auto it = std::find(fixed_edge_point_ids.begin(), fixed_edge_point_ids.end(), edge_point_id);
    if (it == fixed_edge_point_ids.end()) {


        throw std::invalid_argument("edge_point_id is not found id: " + std::to_string(edge_point_id));
    }

    return std::distance(fixed_edge_point_ids.begin(), it);
}

