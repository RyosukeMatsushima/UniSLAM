#include "local_slam.hpp"

LocalSlam::LocalSlam(const CameraModel& camera_model,
                     const std::string& edge_space_dynamics_config_file)
    : camera_model(camera_model),
      edge_space_dynamics(edge_space_dynamics_config_file),
      current_frame_node(cv::Mat::zeros(10, 10, CV_8UC1), WINDOW_SIZE, ANGLE_RESOLUTION) {}

bool LocalSlam::multi_frame_init(const cv::Mat& image,
                                 const Eigen::Vector3f& external_position_data,
                                 const Eigen::Quaternionf& external_orientation_data) {

    frame_count++;
    current_frame_node = FrameNode(image, WINDOW_SIZE, ANGLE_RESOLUTION);
    Pose3D current_frame_pose(external_position_data, external_orientation_data);

    if (key_frames.empty()) {
        key_frames.push_back(std::make_pair(current_frame_node, current_frame_pose));
        return false;
    }

    fix_edges(key_frames.back().first, current_frame_node, key_frames.back().second, current_frame_pose);

    // shuffle fixed edge points for edge space dynamics pose calculation
    current_frame_node.shuffleFixedEdgePoints();

    // check initialization
    // try to calculate the pose of the current frame and the last key frame
    // if the both poses are calculated correctly, the initialization is done
    Pose3D restored_pose(external_position_data, external_orientation_data);
    // move the initial pose to the z-axis direction of current_frame_pose
    restored_pose.translate(current_frame_pose.rotateVectorToWorld(Eigen::Vector3f(0.0f, 0.0f, 1.0f)));

    if (!get_pose(current_frame_node, restored_pose)) return false;


    // check restored_pose
    if (restored_pose.translationalDiffTo(current_frame_pose).norm() > VALID_TRANSLATIONAL_DIFF ||
        restored_pose.rotationalDiffTo(current_frame_pose).norm() > VALID_ROTATIONAL_DIFF) {

        // clean edge space dynamics
        edge_space_dynamics.clear_edges();
        return false;
    }

    return true;
}

bool LocalSlam::update(const cv::Mat& image,
                       Pose3D& pose) {

    frame_count++;
    current_frame_node = FrameNode(image, WINDOW_SIZE, ANGLE_RESOLUTION);

    bool is_key_frame;
    if (!current_frame_node.matchWith(key_frames.back().first, is_key_frame)) {
        std::cout << "Failed to match with the last key frame" << std::endl;
        return false;
    }

    current_frame_node.shuffleFixedEdgePoints();
    if (!get_pose(current_frame_node, pose)) {
        std::cout << "Failed to get pose" << std::endl;
        return false;
    }

    if (!is_key_frame) return true;

    fix_edges(key_frames.back().first, current_frame_node, key_frames.back().second, pose);

    key_frames.push_back(std::make_pair(current_frame_node, pose));

    return true;
}

void LocalSlam::fix_edges(FrameNode& frame_node1,
                          FrameNode& frame_node2,
                          const Pose3D& pose_frame1,
                          const Pose3D& pose_frame2) {

    std::vector<EdgePoint> edge_points = frame_node1.findNewEdgePoints();

    for (auto& edge_point : edge_points) {
        EdgePoint matched_edge_point;
        if (!frame_node2.matchEdge(edge_point, matched_edge_point)) continue;

        int edge_id;
        EdgeNode edge_node = camera_model.getEdgeNode(edge_point);
        EdgeNode matched_edge_node = camera_model.getEdgeNode(matched_edge_point);
        bool result = edge_space_dynamics.add_new_edge(pose_frame1,
                                                       pose_frame2,
                                                       edge_node,
                                                       matched_edge_node,
                                                       edge_id);

        if (!result) continue;

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
        return result;
    } catch (const std::exception& e) {
        std::cout << "Failed to get pose" << std::endl;
        std::cout << e.what() << std::endl;
        return false;
    }
}

std::vector<Line3D> LocalSlam::get_fixed_edges() {
    return edge_space_dynamics.get_edge3ds();
}

//bool LocalSlam::calculate_first_matched_edges(const FrameNode& last_key_frame,
//                                              const FrameNode& current_frame) {
//
//    for (const auto& frame1_edge_point : last_key_frame.findNewEdgePoints()) {
//
//        EdgePoint frame2_edge_point;
//        if (!current_frame.matchEdge(frame1_edge_point, frame2_edge_point)) continue;
//
//        int edge_id;
//        bool result = edge_space_dynamics.add_new_edge(last_key_frame.get_pose(),
//                                                       current_frame.get_pose(),
//                                                       frame1_edge_point,
//                                                       frame2_edge_point,
//                                                       edge_id);
//
//        if (!result) return continue;
//
//        frame1_edge_point.id = edge_id;
//        frame2_edge_point.id = edge_id;
//
//        last_key_frame.addFixedEdgePoint(frame1_edge_point);
//        current_frame.addFixedEdgePoint(frame2_edge_point);
//        }
//    }
//}

void LocalSlam::save_log(const std::string& path_to_dir) {
    DebugView debug_view(current_frame_node.getImg());

    std::cout << "type of base_img: " << key_frames.back().first.getImg().type() << std::endl;
    debug_view.drawEdgePoints(key_frames.back().first.getFixedEdgePoints(), cv::Scalar(0, 0, 255));
    debug_view.drawEdgePoints(current_frame_node.getFixedEdgePoints(), cv::Scalar(0, 255, 0));

    cv::imwrite(path_to_dir + "frame" + std::to_string(frame_count) + ".png", debug_view.getDebugImage());
}
