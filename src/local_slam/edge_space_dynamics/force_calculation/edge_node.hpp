#ifndef EDGE_NODE_HPP
#define EDGE_NODE_HPP

#include <Eigen/Dense>

struct EdgeNode {
    int edge_id;
    Eigen::Vector3f direction_frame_to_edge;
    Eigen::Vector2f edge_direction;
    bool is_valid = true;

    EdgeNode() {
        direction_frame_to_edge = Eigen::Vector3f::Zero();
        edge_direction = Eigen::Vector2f::Zero();
        edge_id = -1;
    }

    EdgeNode(const Eigen::Vector3f& direction_frame_to_edge,
             const Eigen::Vector2f& edge_direction,
             const int& edge_id)
        : direction_frame_to_edge(direction_frame_to_edge),
          edge_direction(edge_direction),
          edge_id(edge_id) {}

};

#endif // EDGE_NODE_HPP
