#ifndef LINE_3D_HPP
#define LINE_3D_HPP

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

class Line3D {
public:

    Line3D(const int id,
           const Eigen::Vector3f start_point,
           const Eigen::Vector3f direction,
           const float length);

    void add_force(const Eigen::Vector3f force,
                   const Eigen::Vector3f torque,
                   const float force_center_from_start);

    Eigen::Vector3f get_point_at(const float distance_from_start) const;

    static bool get_closest_points_between(const Line3D& line1,
                                           const Line3D& line2,
                                           float& distance1,
                                           float& distance2);

    Eigen::Vector3f start_point() const;

    Eigen::Vector3f direction() const;

private:
    int id_;

    Eigen::Vector3f start_point_;

    Eigen::Vector3f direction_;

    float length_;
};

#endif // LINE_3D_HPP
