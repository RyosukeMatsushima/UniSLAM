#ifndef LINE_3D_HPP
#define LINE_3D_HPP

#define CONNECTION_ANGLE_THRESHOLD 0.02f // 1.15 degrees
#define CONNECTION_DISTANCE_THRESHOLD 0.01f
#define CONNECTION_MAX_START_POINT_DISTANCE 0.3f

#include <Eigen/Dense>

class Line3D {
public:

    Line3D(const int id,
           const Eigen::Vector3f start_point,
           const Eigen::Vector3f direction,
           const float length);

    void add_force(const Eigen::Vector3f force,
                   const Eigen::Vector3f torque,
                   const float force_center_from_start);

    void move(const Eigen::Vector3f delta);

    Eigen::Vector3f get_point_at(const float distance_from_start) const;

    static bool get_closest_points_between(const Line3D& line1,
                                           const Line3D& line2,
                                           float& distance1,
                                           float& distance2);

    Line3D clone() const;

    bool connect(const Line3D& other);

    float get_closest_point_to(const Eigen::Vector3f point) const;

    int id() const;

    Eigen::Vector3f start_point() const;

    Eigen::Vector3f direction() const;

    float length() const;

private:
    int id_;

    Eigen::Vector3f start_point_;

    Eigen::Vector3f direction_;

    float length_;
};

#endif // LINE_3D_HPP
