#ifndef EDGE_3D_HPP
#define EDGE_3D_HPP

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

class Edge3D {
public:

    Edge3D(const int id,
           const Eigen::Vector3f start_point,
           const Eigen::Vector3f direction,
           const float length);

    void add_force(const Eigen::Vector3f force,
                   const Eigen::Vector3f torque,
                   const float force_center_from_start);

    Eigen::Vector3f get_point_at(const float distance_from_start) const;

private:
    int id_;

    Eigen::Vector3f start_point_;

    Eigen::Vector3f direction_;

    float length_;
};

#endif // EDGE_3D_HPP
