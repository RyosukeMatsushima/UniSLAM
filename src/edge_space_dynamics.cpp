#include "edge_space_dynamics.hpp"

EdgeSpaceDynamics::EdgeSpaceDynamics() {
}

bool EdgeSpaceDynamics::get_frame_pose(std::vector<EdgeNode> edge_nodes,
                                       Pose3D& frame_pose) {
    for (int i = 0; i < MAX_CAL_ITER; i++) {
        Pose3D current_frame_pose = frame_pose.clone();

        for (int j = 0; j < edge_nodes.size(); j++) {
            EdgeNode edge_node = edge_nodes[j];
            Line3D edge = edges[edge_node.edge_id];
            Force3D force_to_frame, force_to_edge;
            float torque_center_point_for_edge_line;
            bool result = force_calculation(edge,
                                            edge_node,
                                            current_frame_pose,
                                            force_to_frame,
                                            force_to_edge,
                                            torque_center_point_for_edge_line);
            if (!result) {
                continue;
            }

            frame_pose.translate(force_to_frame.force * FRAME_POSE_TRANSLATE_GAIN);
            frame_pose.rotate(force_to_edge.torque * FRAME_POSE_ROTATE_GAIN);
        }
    }
    return true;
}

bool EdgeSpaceDynamics::add_new_edge(Pose3D frame1_pose,
                                     Pose3D frame2_pose,
                                     EdgeNode frame1_edge_node,
                                     EdgeNode frame2_edge_node,
                                     int& edge_id) {
    return true;
}

Pose3D EdgeSpaceDynamics::optimize(Pose3D frame_pose,
                                   std::vector<EdgeNode> edge_nodes) {
    return frame_pose;
}

void EdgeSpaceDynamics::get_edge3ds(std::vector<Line3D>& edge3ds) {
}

int EdgeSpaceDynamics::set_edge3d(Eigen::Vector3f start_point,
                                  Eigen::Vector3f direction,
                                  float length) {
    int edge_id = edges.size();
    Line3D edge3d(edge_id, start_point, direction, length);
    edges.push_back(edge3d);
    edge_ids.push_back(edge_id);
    return edge_id;
}

