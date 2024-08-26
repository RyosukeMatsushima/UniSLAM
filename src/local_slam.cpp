#include "local_slam.hpp"

LocalSlam::LocalSlam(const CameraModel& camera_model,
                     const std::string& edge_space_dynamics_config_file)
    : camera_model(camera_model),
      edge_space_dynamics(edge_space_dynamics_config_file),
      current_frame_node(cv::Mat::zeros(1000, 1000, CV_8UC1), WINDOW_SIZE, ANGLE_RESOLUTION) {}

bool LocalSlam::multi_frame_init(const cv::Mat& image) {

    frame_count++;
    current_frame_node = FrameNode(image, WINDOW_SIZE, ANGLE_RESOLUTION);

    if (key_frames.empty()) {
        std::vector<EdgePoint> edge_points = current_frame_node.findNewEdgePoints();
        for (auto& edge_point : edge_points) {
            edge_point.id = 0; // TODO: modify this. this is a temporary solution to avoid the error at addFixedEdgePoint
            current_frame_node.addFixedEdgePoint(edge_point);
        }

        key_frames.push_back(FrameNodeData(current_frame_node, Pose3D(), Pose3D(), false));

        return false;
    }

    bool is_key_frame;
    if (!current_frame_node.matchWith(key_frames.back().frame_node, is_key_frame)) {
        std::cout << "Failed to match with the last key frame" << std::endl;
        return false;
    }

    if (!is_key_frame) return false;

    // add new edge points to edge space dynamics
    key_frames.back().frame_node.clearFixedEdgePoints();
    current_frame_node.clearFixedEdgePoints();

    add_key_frame(FrameNodeData(current_frame_node, Pose3D(), Pose3D(), false));

    return true;
}

bool LocalSlam::update(const cv::Mat& image,
                       const Pose3D& external_pose_data,
                       const bool use_external_pose_data,
                       const bool calculate_pose,
                       Pose3D& pose) {

    frame_count++;
    current_frame_node = FrameNode(image, WINDOW_SIZE, ANGLE_RESOLUTION);

    // match with the last key frame
    bool is_key_frame;
    if (!current_frame_node.matchWith(key_frames.back().frame_node, is_key_frame)) {
        std::cout << "Failed to match with the last key frame" << std::endl;
        return false;
    }

    // if is_key_frame, add new edge points to edge space dynamics
    if (is_key_frame) {
        std::cout << "add key frame" << std::endl;
        add_key_frame(FrameNodeData(current_frame_node, external_pose_data, pose, use_external_pose_data));
    }

    // calculate the pose of the current frame
    if (calculate_pose) {
        if (!get_pose(current_frame_node, pose)) {
            std::cout << "Failed to get pose" << std::endl;
            return false;
        }
    }

    return true;
}

void LocalSlam::optimize(const int iteration) {
    for (int i = 0; i < iteration; i++) {
        for (int key_frame_index = 0; key_frame_index < key_frames.size(); key_frame_index++) {
            std::vector<EdgeNode> edge_nodes;
            for (const auto& edge_point : key_frames[key_frame_index].frame_node.getFixedEdgePoints()) {
                EdgeNode edge_node = camera_model.getEdgeNode(edge_point);
                edge_node.is_valid = true;
                edge_nodes.push_back(edge_node);
            }

            edge_space_dynamics.optimize(key_frames[key_frame_index].calculated_pose,
                                         edge_nodes,
                                         key_frames[key_frame_index].external_pose_data,
                                         key_frames[key_frame_index].use_external_pose_data);

            // remove invalid edge points
            for (const auto& edge_node : edge_nodes) {
                if (!edge_node.is_valid) {
                    key_frames[key_frame_index].frame_node.removeFixedEdgePoint(edge_node.edge_id);
                }
            }
        }
    }
}

void LocalSlam::fix_edges(FrameNode& frame_node1,
                          FrameNode& frame_node2,
                          const Pose3D& pose_frame1,
                          const Pose3D& pose_frame2) {

    std::vector<EdgePoint> edge_points = frame_node1.findNewEdgePoints();

    rejected_edge_points.clear();
    unmatched_edge_points.clear();

    for (auto& edge_point : edge_points) {
        EdgePoint matched_edge_point;
        if (!frame_node2.matchEdge(edge_point, matched_edge_point)) {
            unmatched_edge_points.push_back(edge_point);
            continue;
        }

        int edge_id;
        EdgeNode edge_node = camera_model.getEdgeNode(edge_point);
        EdgeNode matched_edge_node = camera_model.getEdgeNode(matched_edge_point);
        bool result = edge_space_dynamics.add_new_edge(pose_frame1,
                                                       pose_frame2,
                                                       edge_node,
                                                       matched_edge_node,
                                                       edge_id);

        if (!result) {
            rejected_edge_points.push_back(std::make_pair(edge_point, matched_edge_point));
            continue;
        }

        edge_point.id = edge_id;
        matched_edge_point.id = edge_id;

        frame_node1.addFixedEdgePoint(edge_point);
        frame_node2.addFixedEdgePoint(matched_edge_point);
    }
}

bool LocalSlam::get_pose(const FrameNode& frame_node,
                         Pose3D& pose) {

    std::vector<EdgeNode> edge_nodes;
    for (const auto& edge_point : frame_node.getFixedEdgePoints()) {
        edge_nodes.push_back(camera_model.getEdgeNode(edge_point));
    }

    try {
        bool result = edge_space_dynamics.get_frame_pose(edge_nodes,
                                                         VALID_EDGE_NODES_RATIO_THRESHOLD,
                                                         pose);

        // TODO: move valid edge point to the biginning of the vector in frame_node

        for (const auto& edge_node : edge_nodes) {
            key_frames.back().frame_node.moveFixedEdgePointToBack(edge_node.edge_id);
        }

        return result;
    } catch (const std::exception& e) {
        std::cout << "Failed to get pose" << std::endl;
        std::cout << e.what() << std::endl;
        return false;
    }
}

void LocalSlam::add_key_frame(FrameNodeData frame_node_data) {
    std::vector<EdgePoint> edge_points = key_frames.back().frame_node.findNewEdgePoints();

    unmatched_edge_points.clear();
    for (auto& edge_point : edge_points) {
        EdgePoint matched_edge_point;
        if (!frame_node_data.frame_node.matchEdge(edge_point, matched_edge_point)) {
            unmatched_edge_points.push_back(edge_point);
            continue;
        }

        EdgeNode edge_node = camera_model.getEdgeNode(edge_point);
        EdgeNode matched_edge_node = camera_model.getEdgeNode(matched_edge_point);

        int edge_id = edge_space_dynamics.set_edge3d(edge_node,
                                                     frame_node_data.calculated_pose,
                                                     1.0);

        edge_point.id = edge_id;
        matched_edge_point.id = edge_id;

        key_frames.back().frame_node.addFixedEdgePoint(edge_point);
        frame_node_data.frame_node.addFixedEdgePoint(matched_edge_point);
    }

    key_frames.push_back(frame_node_data);
}

std::vector<Line3D> LocalSlam::get_fixed_edges() {
    return edge_space_dynamics.get_edge3ds();
}

void LocalSlam::save_log(const std::string& path_to_dir) {
    // debug view on current frame
    VslamDebugView debug_view(current_frame_node.getImg());
    debug_view.drawEdgePoints(key_frames.back().frame_node.getFixedEdgePoints(), cv::Scalar(0, 0, 255));
    debug_view.drawEdgePoints(current_frame_node.getFixedEdgePoints(), cv::Scalar(0, 255, 0));
    cv::imwrite(path_to_dir + "frame" + std::to_string(frame_count) + ".png", debug_view.getDebugImage());

    // debug view on key frame
    VslamDebugView debug_view_on_key_frame(key_frames.back().frame_node.getImg());
    debug_view_on_key_frame.drawEdgePoints(key_frames.back().frame_node.getFixedEdgePoints(), cv::Scalar(0, 0, 255));
    debug_view_on_key_frame.drawEdgePoints(current_frame_node.getFixedEdgePoints(), cv::Scalar(0, 255, 0));
    
    // draw rejected edge points
//    for (const auto& rejected_edge_point : rejected_edge_points) {
//        debug_view_on_key_frame.drawEdgePoints({rejected_edge_point.first, rejected_edge_point.second}, cv::Scalar(0, 255, 255));
//        debug_view_on_key_frame.drawEdgePoints({rejected_edge_point.second, rejected_edge_point.second}, cv::Scalar(0, 100, 100));
//    }

    // draw unmatched edge points
    std::cout << "unmatched edge points: " << unmatched_edge_points.size() << std::endl;
    debug_view_on_key_frame.drawEdgePoints(unmatched_edge_points, cv::Scalar(100, 100, 0));

    for (const auto& edge_3d : edge_space_dynamics.get_edge3ds()) {
        debug_view_on_key_frame.drawEdge3D(edge_3d, key_frames.back().external_pose_data, camera_model.getCameraMatrix(), cv::Scalar(255, 255, 0));
    }
    cv::imwrite(path_to_dir + "key_frame" + std::to_string(frame_count) + ".png", debug_view_on_key_frame.getDebugImage());

    // third person view
    VslamDebugView debug_view_third_person_view(cv::Mat::zeros(1000, 1000, current_frame_node.getImg().type()));

    Pose3D camera_pose;
    camera_pose.translate(Eigen::Vector3f(1, -0.6, -1.5));
    camera_pose.rotate(Eigen::Vector3f(-0.2, -0.7, 0));
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1000, 0, 500, 0, 1000, 500, 0, 0, 1);

    for (const auto& edge_3d : edge_space_dynamics.get_edge3ds()) {
        debug_view_third_person_view.drawEdge3D(edge_3d, camera_pose, camera_matrix, cv::Scalar(255, 255, 0));
    }
    cv::imwrite(path_to_dir + "third_person_view" + std::to_string(frame_count) + ".png", debug_view_third_person_view.getDebugImage());
}
