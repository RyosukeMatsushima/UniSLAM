#include "local_slam.hpp"

LocalSlam::LocalSlam() {
}

bool LocalSlam::multi_frame_init(const cv::Mat& image) {

//    FrameNode frame_node(image);
//    if (frame_node.findNewEdge().empty()) return false;
//    key_frames.push_back(frame_node);
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

//bool LocalSlam::get_pose(const FrameNode& frame_node,
//                         Pose3D& pose) {
//
//    for (const auto& edge_point : key_frames.back().getFixedEdgePoints()) {
//        EdgePoint matched_edge_point;
//        frame_node.matchEdge(edge_point, matched_edge_point);
//        matched_edge_point.edge_id = edge_point.edge_id;
//        frame_node.addFixedEdgePoint(matched_edge_point);
//    }
//
//    return edge_space_dynamics.get_frame_pose(frame_node.getFixedEdgePoints(), pose);
//}

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

