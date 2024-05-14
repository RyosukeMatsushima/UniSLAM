#include "line_3d.hpp"

Line3D::Line3D(const int id,
               const Eigen::Vector3f start_point,
               const Eigen::Vector3f direction,
               const float length)
    : id_(id),
      start_point_(start_point),
      direction_(direction),
      length_(length)
{
}

void Line3D::add_force(const Eigen::Vector3f force,
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

Eigen::Vector3f Line3D::get_point_at(const float distance_from_start) const
{
    return start_point_ + distance_from_start * direction_;
}

bool Line3D::get_closest_points_between(const Line3D& line1,
                                        const Line3D& line2,
                                        float& distance1,
                                        float& distance2)
{
    Eigen::Vector3f w0 = line1.start_point_ - line2.start_point_;
    float a = line1.direction_.dot(line1.direction_);
    float b = line1.direction_.dot(line2.direction_);
    float c = line2.direction_.dot(line2.direction_);
    float d = line1.direction_.dot(w0);
    float e = line2.direction_.dot(w0);

    float denominator = a * c - b * b;
    if (denominator < 1e-6) {
        // Lines are parallel
        return false;
    }

    distance1 = (b * e - c * d) / denominator;
    distance2 = (a * e - b * d) / denominator;

    Eigen::Vector3f p1 = line1.get_point_at(distance1);
    Eigen::Vector3f p2 = line2.get_point_at(distance2);

    return true;
}
