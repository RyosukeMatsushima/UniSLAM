#ifndef FORCE_CALCULATION_HPP
#define FORCE_CALCULATION_HPP

#include <Eigen/Dense>

#include "edge_node.hpp"
#include "line_3d.hpp"
#include "pose_3d.hpp"

struct Force3D {
    Eigen::Vector3f force; // Force vector
    Eigen::Vector3f torque; // Torque vector

    Force3D() {
        force = Eigen::Vector3f(0, 0, 0);
        torque = Eigen::Vector3f(0, 0, 0);
    }

    Force3D(const Eigen::Vector3f &force, const Eigen::Vector3f &torque) {
        this->force = force;
        this->torque = torque;
    }
};

bool force_calculation(const Line3D &edge,
                       const EdgeNode &edge_node,
                       const Pose3D &frame_pose,
                       Force3D &force_to_frame,
                       Force3D &force_to_edge,
                       float &torque_center_point_for_edge_line);

#endif // FORCE_CALCULATION_HPP
