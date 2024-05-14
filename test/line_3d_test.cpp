#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "line_3d.hpp"

// Test the Line3D class
TEST(Line3DTest, TestLine3D)
{
    // Create an Line3D object
    Line3D line(0, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(1, 0, 0), 1.0);

    // Add a force to the line
    line.add_force(Eigen::Vector3f(0, 0, 0),
                   Eigen::Vector3f(0, 0, M_PI / 2.0f),
                   1.0);

    Eigen::Vector3f start_point = line.get_point_at(0);
    ASSERT_NEAR(start_point(0), 1, 1e-6);
    ASSERT_NEAR(start_point(1), -1, 1e-6);
    ASSERT_NEAR(start_point(2), 0, 1e-6);

    Eigen::Vector3f end_point = line.get_point_at(1);
    ASSERT_NEAR(end_point(0), 1, 1e-6);
    ASSERT_NEAR(end_point(1), 0, 1e-6);
    ASSERT_NEAR(end_point(2), 0, 1e-6);
}
