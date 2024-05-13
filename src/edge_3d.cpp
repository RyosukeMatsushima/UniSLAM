#include "edge_3d.hpp"

Edge3D::Edge3D(const int id,
               const Eigen::Vector3f start_point,
               const Eigen::Vector3f direction,
               const float length)
    : id_(id),
      start_point_(start_point),
      direction_(direction),
      length_(length)
{
}

void Edge3D::add_force(const Eigen::Vector3f force,
                       const Eigen::Vector3f torque,
                       const float force_center_from_start)
{
    start_point_ += force;

    // Compute rotation axis and angle from torque vector
    double angle = torque.norm();
    if (angle > 1e-6) {
        Eigen::Vector3f axis = torque.normalized();

        // Compute rotation matrix
        Eigen::Matrix3f R = Eigen::AngleAxisf(angle, axis).toRotationMatrix();
        Eigen::Vector3f updated_direction = R * direction_;

        // Move start_point_ by force_center_from_start
        Eigen::Vector3f force_center_point = start_point_ + force_center_from_start * direction_;
        start_point_ = force_center_point - updated_direction * force_center_from_start;

        // Rotate direction vector
        direction_ = updated_direction;
    }
}

Eigen::Vector3f Edge3D::get_point_at(const float distance_from_start) const
{
    return start_point_ + distance_from_start * direction_;
}

