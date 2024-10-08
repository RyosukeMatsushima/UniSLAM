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

    // calculate the pose of the current frame
    if (calculate_pose) {
        if (!get_pose(current_frame_node, pose)) {
            std::cout << "Failed to get pose" << std::endl;
            return false;
        }
    }

    // is_key_frame = true if use_external_pose_data is true and last n key frames are not use_external_pose_data
    int n = 2;
    if (use_external_pose_data) {
        is_key_frame = true;
        for (int i = 1; i <= n; i++) {
            if (key_frames.size() < i) {
                break;
            }
            if (key_frames[key_frames.size() - i].use_external_pose_data) {
                is_key_frame = false;
                break;
            }
        }
    }

    // if is_key_frame, add new edge points to edge space dynamics
    if (is_key_frame) {
        std::cout << "add key frame" << std::endl;
        add_key_frame(FrameNodeData(current_frame_node, external_pose_data, pose, use_external_pose_data));
    }

    return true;
}

void LocalSlam::optimize(const int iteration) {
    optimize_count++;

    for (int i = 0; i < iteration; i++) {
        for (int key_frame_index = 0; key_frame_index < key_frames.size(); key_frame_index++) {
            std::vector<EdgePoint> edge_points = key_frames[key_frame_index].frame_node.getFixedEdgePoints();
            std::vector<EdgeNode> edge_nodes;
            for (const auto& edge_point : edge_points) {
                EdgeNode edge_node = camera_model.getEdgeNode(edge_point);
                edge_node.is_valid = true;
                edge_nodes.push_back(edge_node);
            }

            edge_space_dynamics.optimize(key_frames[key_frame_index].calculated_pose,
                                         edge_nodes,
                                         key_frames[key_frame_index].external_pose_data,
                                         key_frames[key_frame_index].use_external_pose_data);

            // update fixed edge points
            for (int i = 0; i < edge_points.size(); i++) {
                if (edge_nodes[i].edge_id != edge_points[i].id) {
                    did_changed_edge_id(edge_points[i].id, edge_nodes[i].edge_id);
                }

                if (!edge_nodes[i].is_valid) {
                    key_frames[key_frame_index].frame_node.removeFixedEdgePoint(edge_nodes[i].edge_id);
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

bool LocalSlam::get_pose(FrameNode& frame_node,
                         Pose3D& pose) {

    std::vector<EdgePoint> edge_points = frame_node.getFixedEdgePoints();
    std::vector<EdgeNode> edge_nodes;
    for (const auto& edge_point : edge_points) {
        edge_nodes.push_back(camera_model.getEdgeNode(edge_point));
    }

    try {
        bool result = edge_space_dynamics.get_frame_pose(edge_nodes,
                                                         VALID_EDGE_NODES_RATIO_THRESHOLD,
                                                         pose);

        // TODO: move valid edge point to the biginning of the vector in frame_node
        frame_node.clearFixedEdgePoints();
        std::vector<EdgePoint> valid_edge_points;
        std::vector<EdgePoint> invalid_edge_points;
        for (int i = 0; i < edge_points.size(); i++) {
            edge_points[i].id = edge_nodes[i].edge_id;
            if (edge_nodes[i].is_valid) {
                valid_edge_points.push_back(edge_points[i]);
            } else {
                invalid_edge_points.push_back(edge_points[i]);
            }
        }

        for (const auto& edge_point : valid_edge_points) {
            frame_node.addFixedEdgePoint(edge_point);
        }
        for (const auto& edge_point : invalid_edge_points) {
            frame_node.addFixedEdgePoint(edge_point);
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

    std::cout << "edge_points.size() = " << edge_points.size() << std::endl;
    found_edge_points = edge_points;

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

void LocalSlam::did_changed_edge_id(const int before_edge_id,
                                    const int after_edge_id) {
    for (int i = 0; i < key_frames.size(); i++) {
        std::vector<EdgePoint> edge_points = key_frames[i].frame_node.getFixedEdgePoints();
        key_frames[i].frame_node.clearFixedEdgePoints();

        for (auto& edge_point : edge_points) {
            if (edge_point.id == before_edge_id) {
                edge_point.id = after_edge_id;
            }
            key_frames[i].frame_node.addFixedEdgePoint(edge_point);
        }
    }
}

std::vector<Line3D> LocalSlam::get_fixed_edges() {
    return edge_space_dynamics.get_edge3ds();
}

VslamDebugView LocalSlam::get_current_debug_view(std::string& file_name) {
    VslamDebugView debug_view(current_frame_node.getImg());
    debug_view.drawEdgePoints(found_edge_points, cv::Scalar(225, 255, 0));
    debug_view.drawEdgePoints(key_frames.back().frame_node.getFixedEdgePoints(), cv::Scalar(225, 0, 0));
    debug_view.drawEdgePoints(current_frame_node.getFixedEdgePoints(), cv::Scalar(0, 255, 0));

    file_name = "current_frame frame: " + std::to_string(frame_count) + " optimize: " + std::to_string(optimize_count) + ".png";
    return debug_view;
}

VslamDebugView LocalSlam::get_key_frame_debug_view(std::string& file_name) {
    VslamDebugView debug_view(key_frames.back().frame_node.getImg());
    debug_view.drawEdgePoints(found_edge_points, cv::Scalar(225, 255, 0));
    debug_view.drawEdgePoints(key_frames.back().frame_node.getFixedEdgePoints(), cv::Scalar(225, 0, 0));
    debug_view.drawEdgePoints(current_frame_node.getFixedEdgePoints(), cv::Scalar(0, 255, 0));
    debug_view.drawEdgePoints(unmatched_edge_points, cv::Scalar(0, 0, 255));

    for (const auto& edge_line : edge_space_dynamics.get_edge3ds()) {
        debug_view.drawEdge3D(edge_line, key_frames.back().calculated_pose, camera_model.getCameraMatrix(), cv::Scalar(255, 0, 0));
    }

    for (const auto& edge_point : key_frames.back().frame_node.getFixedEdgePoints()) {
        try {
            Line3D edge_3d = edge_space_dynamics.get_edge3d(edge_point.id);
            debug_view.drawEdge3D(edge_3d, key_frames.back().calculated_pose, camera_model.getCameraMatrix(), cv::Scalar(0, 0, 255));
        } catch (const std::exception& e) {
            std::cout << e.what() << std::endl;
        }
    }

    for (const auto& edge_point : current_frame_node.getFixedEdgePoints()) {
        try {
            Line3D edge_3d = edge_space_dynamics.get_edge3d(edge_point.id);
            debug_view.drawEdge3D(edge_3d, key_frames.back().calculated_pose, camera_model.getCameraMatrix(), cv::Scalar(0, 255, 0));
        } catch (const std::exception& e) {
            std::cout << e.what() << std::endl;
        }
    }

    file_name = "key_frame frame: " + std::to_string(frame_count) + " optimize: " + std::to_string(optimize_count) + ".png";
    return debug_view;
}

VslamDebugView LocalSlam::get_third_person_view(const Pose3D& camera_pose,
                                                cv::Mat& base_image,
                                                cv::Mat& camera_matrix,
                                                std::string& file_name) {
    VslamDebugView debug_view_third_person_view(base_image);

    for (const auto& edge_3d : edge_space_dynamics.get_edge3ds()) {
        debug_view_third_person_view.drawEdge3D(edge_3d, camera_pose, camera_matrix, cv::Scalar(255, 0, 0));
    }

    for (const auto& edge_point : key_frames.back().frame_node.getFixedEdgePoints()) {
        try {
            Line3D edge_3d = edge_space_dynamics.get_edge3d(edge_point.id);
            debug_view_third_person_view.drawEdge3D(edge_3d, camera_pose, camera_matrix, cv::Scalar(0, 0, 255));
        } catch (const std::exception& e) {
            std::cout << e.what() << std::endl;
        }
    }

    for (const auto& edge_point : current_frame_node.getFixedEdgePoints()) {
        try {
            Line3D edge_3d = edge_space_dynamics.get_edge3d(edge_point.id);
            debug_view_third_person_view.drawEdge3D(edge_3d, camera_pose, camera_matrix, cv::Scalar(0, 255, 0));
        } catch (const std::exception& e) {
            std::cout << e.what() << std::endl;
        }
    }

    // draw current frame pose
    //debug_view_third_person_view.drawPose3D(key_frames.back().calculated_pose, camera_pose, camera_matrix, cv::Scalar(255, 0, 255), 0.3);
    //debug_view_third_person_view.drawPose3D(key_frames.back().external_pose_data, camera_pose, camera_matrix, cv::Scalar(0, 255, 255), 0.3);

    for (int i = 0; i < key_frames.size(); i++) {
        FrameNodeData key_frame = key_frames[key_frames.size() - i - 1];
        debug_view_third_person_view.drawPose3D(key_frame.calculated_pose, camera_pose, camera_matrix, cv::Scalar(255, 0, 0), 0.2);

        if (key_frame.use_external_pose_data) debug_view_third_person_view.drawPose3D(key_frame.external_pose_data, camera_pose, camera_matrix, cv::Scalar(0, 255, 0), 0.2);
    }

    file_name = "third_person_view frame: " + std::to_string(frame_count) + " optimize: " + std::to_string(optimize_count) + ".png";
    return debug_view_third_person_view;
}

