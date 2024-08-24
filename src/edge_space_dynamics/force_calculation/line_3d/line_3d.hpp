#ifndef LINE_3D_HPP
#define LINE_3D_HPP

#define CONNECTION_ANGLE_THRESHOLD 0.2f
#define CONNECTION_DISTANCE_THRESHOLD 0.1f
#define CONNECTION_MAX_START_POINT_DISTANCE 0.5f //TODO: parameters as input

#include <Eigen/Dense>

#include "vector_average.hpp"

class Line3D {
public:

    Line3D(const int id,
           const Eigen::Vector3f start_point,
           const Eigen::Vector3f direction,
           const float length,
           const int stock_size = 10,
           const float fixed_start_point_variance_threshold = 0.0f,
           const float fixed_direction_variance_threshold = 0.0f);

    void add_force(const Eigen::Vector3f force,
                   const Eigen::Vector3f torque,
                   const float force_center_from_start);

    // TODO: remove this method. this can achieve by add_force
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

    bool is_fixed() const;

    void clear_history();

    int update_count() const { return update_count_; }

private:
    int id_;

    Eigen::Vector3f start_point_;

    Eigen::Vector3f direction_;

    float length_;

    VectorAverage startPointAverage;

    VectorAverage directionAverage;

    unsigned int update_count_ = 0;

    float fixed_start_point_variance_threshold;

    float fixed_direction_variance_threshold;
};

#endif // LINE_3D_HPP
