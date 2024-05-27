#include <gtest/gtest.h>

#include "force_calculation.hpp"

TEST(ForceCalculationTest, force_calculation_test) {

    Line3D edge(0,
                Eigen::Vector3f(0, 1, 1),
                Eigen::Vector3f(1, 0, 0),
                1);

    EdgeNode edge_node(Eigen::Vector3f(0, 0, 1),
                       Eigen::Vector2f(1, 0),
                       0);

    Pose3D pose;

    Force3D force = force_calculation(edge, edge_node, pose);

    Force3D expected_force(Eigen::Vector3f(0, 1, 0), Eigen::Vector3f(-M_PI/4, 0, 0));

    float force_threshold = 0.0001;
    EXPECT_NEAR(force.force(0), expected_force.force(0), force_threshold);
    EXPECT_NEAR(force.force(1), expected_force.force(1), force_threshold);
    EXPECT_NEAR(force.force(2), expected_force.force(2), force_threshold);

    float torque_threshold = 0.0001;
    EXPECT_NEAR(force.torque(0), expected_force.torque(0), torque_threshold);
    EXPECT_NEAR(force.torque(1), expected_force.torque(1), torque_threshold);
    EXPECT_NEAR(force.torque(2), expected_force.torque(2), torque_threshold);
}

TEST(ForceCalculationTest, torque_calculation_test) {

    Line3D edge(0,
                Eigen::Vector3f(0, 0, 1),
                Eigen::Vector3f(1, 0, 0),
                1);

    EdgeNode edge_node(Eigen::Vector3f(0, 0, 1),
                       Eigen::Vector2f(1, 1),
                       0);

    Pose3D pose;

    Force3D force = force_calculation(edge, edge_node, pose);

    Force3D expected_force(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, -M_PI/4));

    float force_threshold = 0.0001;
    EXPECT_NEAR(force.force(0), expected_force.force(0), force_threshold);
    EXPECT_NEAR(force.force(1), expected_force.force(1), force_threshold);
    EXPECT_NEAR(force.force(2), expected_force.force(2), force_threshold);

    float torque_threshold = 0.0001;
    EXPECT_NEAR(force.torque(0), expected_force.torque(0), torque_threshold);
    EXPECT_NEAR(force.torque(1), expected_force.torque(1), torque_threshold);
    EXPECT_NEAR(force.torque(2), expected_force.torque(2), torque_threshold);
}

