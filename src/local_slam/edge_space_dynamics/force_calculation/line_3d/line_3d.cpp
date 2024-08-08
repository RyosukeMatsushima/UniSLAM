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
    direction_ = direction_.normalized();
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
        direction_ = direction_.normalized();
    }
}

void Line3D::move(const Eigen::Vector3f delta)
{
    start_point_ += delta;
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

Line3D Line3D::clone() const
{
    return Line3D(id_, start_point_, direction_, length_);
}

bool Line3D::connect(const Line3D& other)
{

    // Check if the lines are parallel
    if (std::acos(direction_.dot(other.direction_)) > CONNECTION_ANGLE_THRESHOLD) {
        return false;
    }

    // Check if the start points are close enough
    float closest_point_to_other_start_line = get_closest_point_to(other.start_point_);

    if (closest_point_to_other_start_line > length_ + CONNECTION_MAX_START_POINT_DISTANCE) {
        return false;
    }

    if (closest_point_to_other_start_line < -(other.length_ + CONNECTION_MAX_START_POINT_DISTANCE)) {
        return false;
    }

    // Check if the other line is close enough
    float distance_to_other_start_line = (get_point_at(closest_point_to_other_start_line) - other.start_point_).norm();

    if (distance_to_other_start_line > CONNECTION_DISTANCE_THRESHOLD) {
        return false;
    }

    // Update the line
    Eigen::Vector3f new_start_point = start_point_;
    Eigen::Vector3f new_direction = direction_;
    float new_length = length_;

    // use other if other start point is behind the start point of this line
    if (closest_point_to_other_start_line < 0) {
        new_start_point = other.start_point_;
    }

    // use other if other end point is over the end point of this line
    float distance_to_end_point = (get_point_at(length_) - new_start_point).norm();
    float distance_to_other_end_point = (other.get_point_at(other.length_) - new_start_point).norm();

    new_length = std::max(distance_to_end_point, distance_to_other_end_point);

    start_point_ = new_start_point;
    direction_ = new_direction;
    length_ = new_length;

    return true;
}

float Line3D::get_closest_point_to(const Eigen::Vector3f point) const
{
    Eigen::Vector3f w = point - start_point_;
    return w.dot(direction_.normalized());
}

int Line3D::id() const
{
    return id_;
}

Eigen::Vector3f Line3D::start_point() const
{
    return start_point_;
}

Eigen::Vector3f Line3D::direction() const
{
    return direction_;
}

float Line3D::length() const
{
    return length_;
}
