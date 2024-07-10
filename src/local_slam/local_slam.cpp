#include "local_slam.hpp"

LocalSlam::LocalSlam(const CameraModel& camera_model) : camera_model(camera_model) {}

bool LocalSlam::multi_frame_init(const cv::Mat& image,
                                 const Eigen::Vector3f& external_position_data,
                                 const Eigen::Quaternionf& external_orientation_data) {

    FrameNode frame_node(image, WINDOW_SIZE, ANGLE_RESOLUTION);
    Pose3D current_frame_pose(external_position_data, external_orientation_data);

    if (key_frames.empty()) {
        key_frames.push_back(std::make_pair(frame_node, current_frame_pose));
        return false;
    }

    fix_edges(key_frames.back().first, frame_node, key_frames.back().second, current_frame_pose);

    // check initialization
    // try to calculate the pose of the current frame and the last key frame
    // if the both poses are calculated correctly, the initialization is done
    Pose3D restored_pose(external_position_data, external_orientation_data);
    // move the initial pose to the z-axis direction of current_frame_pose
    restored_pose.translate(current_frame_pose.rotateVectorToWorld(Eigen::Vector3f(0.0f, 0.0f, 1.0f)));

    if (!get_pose(frame_node, restored_pose)) return false;

    // check restored_pose
    if (restored_pose.translationalDiffTo(current_frame_pose).norm() > VALID_TRANSLATIONAL_DIFF ||
        restored_pose.rotationalDiffTo(current_frame_pose).norm() > VALID_ROTATIONAL_DIFF) {

        // clean edge space dynamics
        edge_space_dynamics = EdgeSpaceDynamics();
        return false;
    }

    return true;
}

bool LocalSlam::update(const cv::Mat& image,
                       Pose3D& pose) {

//    FrameNode current_frame_node(image);
//
//    bool success_to_get_pose = get_pose(current_frame_node, pose);
//
//    if (!success_to_get_pose) return false;
//
//    // if the current frame is a key frame, add the frame to the key frames
//    if (current_frame_node.isKeyFrame()) {
//
//        calculate_first_matched_edges(key_frames.back(), current_frame_node);
//        key_frames.push_back(current_frame_node);
//    
//    }
//
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
        bool result = edge_space_dynamics.add_new_edge(pose_frame1,
                                                       pose_frame2,
                                                       camera_model.getEdgeNode(edge_point),
                                                       camera_model.getEdgeNode(matched_edge_point),
                                                       edge_id);

        //if (!result) continue;

        edge_point.edge_id = edge_id;
        matched_edge_point.edge_id = edge_id;

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
        return edge_space_dynamics.get_frame_pose(edge_nodes,
                                                  VALID_EDGE_NODES_RATIO_THRESHOLD,
                                                  pose);
    } catch (const std::exception& e) {
        return false;
    }
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
//        frame1_edge_point.edge_id = edge_id;
//        frame2_edge_point.edge_id = edge_id;
//
//        last_key_frame.addFixedEdgePoint(frame1_edge_point);
//        current_frame.addFixedEdgePoint(frame2_edge_point);
//        }
//    }
//}

