#ifndef EDGE_NODE_HPP
#define EDGE_NODE_HPP

#include <Eigen/Dense>

struct EdgeNode {
    int edge_id;
    Eigen::Vector3f direction_frame_to_edge;
    Eigen::Vector2f edge_direction;

    EdgeNode(const Eigen::Vector3f& direction_frame_to_edge,
             const Eigen::Vector2f& edge_direction,
             const int& edge_id)
        : direction_frame_to_edge(direction_frame_to_edge),
          edge_direction(edge_direction),
          edge_id(edge_id) {}

};

#endif // EDGE_NODE_HPP
