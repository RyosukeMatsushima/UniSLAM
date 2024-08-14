#ifndef FORCE_CALCULATION_HPP
#define FORCE_CALCULATION_HPP

#include "edge_node.hpp"
#include "line_3d.hpp"
#include "pose_3d.hpp"
#include <Eigen/Core>

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

    void add(const Force3D &force) {
        this->force += force.force;
        this->torque += force.torque;
    }

    Force3D clone() const {
        return Force3D(force, torque);
    }
};

class ForceCalculation {
public:
    ForceCalculation(const Line3D &edge, const EdgeNode &edge_node, const Pose3D &frame_pose);
    bool calculate();

    Force3D getForceToFrame() const;
    Force3D getForceToEdge() const;
    float getTorqueCenterPointForEdgeLine() const;

private:
    Line3D createObservedLine() const;
    bool calculateClosestPoints(const Line3D &observed_line, float &distance_edge, float &distance_observed_line) const;
    Eigen::Vector3f calculateForce(const Line3D &observed_line, float distance_edge, float distance_observed_line) const;
    Eigen::Vector3f calculateTorque(const Line3D &observed_line, float distance_edge) const;
    Eigen::Vector3f calculateTorqueToAdjust(const Line3D &observed_line) const;

    const Line3D &edge;
    const EdgeNode &edge_node;
    const Pose3D &frame_pose;

    Force3D force_to_frame;
    Force3D force_to_edge;
    float torque_center_point_for_edge_line;
};

#endif // FORCE_CALCULATION_HPP
