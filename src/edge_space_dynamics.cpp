#include "edge_space_dynamics.hpp"

EdgeSpaceDynamics::EdgeSpaceDynamics() {
}

bool EdgeSpaceDynamics::get_frame_pose(std::vector<EdgeNode> edge_nodes,
                                       Pose3D& frame_pose) {
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
    return 0;
}

