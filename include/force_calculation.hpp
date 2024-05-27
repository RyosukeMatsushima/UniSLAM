#ifndef FORCE_CALCULATION_HPP
#define FORCE_CALCULATION_HPP

#include <Eigen/Dense>

#include "edge_node.hpp"
#include "line_3d.hpp"
#include "pose_3d.hpp"

struct Force3D {
    Eigen::Vector3f force; // Force vector
    Eigen::Vector3f torque; // Torque vector

    Force3D(Eigen::Vector3f f, Eigen::Vector3f t) : force(f), torque(t) {}
};

Force3D force_calculation(const Line3D &edge,
                          const EdgeNode &edge_node,
                          const Pose3D &pose);

#endif // FORCE_CALCULATION_HPP
