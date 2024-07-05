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

// Test the get_closest_points_between function
TEST(Line3DTest, TestDistanceBetween)
{
    // Create two Line3D objects
    Line3D line1(0, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(1, 0, 0), 1.0);
    Line3D line2(1, Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(0, 1, 0), 1.0);

    // Compute the distance between the two lines
    float distance1, distance2;
    bool result = Line3D::get_closest_points_between(line1, line2, distance1, distance2);
    ASSERT_TRUE(result);
    ASSERT_NEAR(distance1, 0, 1e-6);
    ASSERT_NEAR(distance2, 0, 1e-6);

    float distance_between = (line1.get_point_at(distance1) - line2.get_point_at(distance2)).norm();
    ASSERT_NEAR(distance_between, 1, 1e-6);
}

TEST(Line3DTest, MoveStartPoint)
{
    // Create an Line3D object
    Line3D line(0, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(1, 0, 0), 1.0);

    // Move the start point
    line.move(Eigen::Vector3f(1, 1, 1));

    Eigen::Vector3f start_point = line.start_point();
    ASSERT_EQ(start_point(0), 1);
    ASSERT_EQ(start_point(1), 1);
    ASSERT_EQ(start_point(2), 1);

    Eigen::Vector3f direction = line.direction();
    ASSERT_EQ(direction(0), 1);
    ASSERT_EQ(direction(1), 0);
    ASSERT_EQ(direction(2), 0);
}

TEST(Line3DTest, Clone)
{
    Line3D line(0, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(1, 0, 0), 1.0);

    Line3D line_clone = line.clone();

    ASSERT_EQ(line.id(), line_clone.id());
    ASSERT_EQ(line.start_point()(0), line_clone.start_point()(0));
    ASSERT_EQ(line.start_point()(1), line_clone.start_point()(1));
    ASSERT_EQ(line.start_point()(2), line_clone.start_point()(2));
    ASSERT_EQ(line.direction()(0), line_clone.direction()(0));
    ASSERT_EQ(line.direction()(1), line_clone.direction()(1));
    ASSERT_EQ(line.direction()(2), line_clone.direction()(2));
    ASSERT_EQ(line.length(), line_clone.length());

    // check that the clone is a deep copy
    line.move(Eigen::Vector3f(1, 1, 1));
    ASSERT_NE(line.start_point()(0), line_clone.start_point()(0));
    ASSERT_NE(line.start_point()(1), line_clone.start_point()(1));
    ASSERT_NE(line.start_point()(2), line_clone.start_point()(2));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
