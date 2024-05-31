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

    Force3D force_to_frame, force_to_edge;

    float torque_center_point_for_edge_line;

    force_calculation(edge, edge_node, pose, force_to_frame, force_to_edge, torque_center_point_for_edge_line);

    Eigen::Vector3f expected_force_to_frame = Eigen::Vector3f(0, 1, 0);
    Eigen::Vector3f expected_torque_to_frame = Eigen::Vector3f(-std::sin(M_PI/4), 0, 0);

    float force_threshold = 0.0001;
    EXPECT_NEAR(force_to_frame.force(0), expected_force_to_frame(0), force_threshold);
    EXPECT_NEAR(force_to_frame.force(1), expected_force_to_frame(1), force_threshold);
    EXPECT_NEAR(force_to_frame.force(2), expected_force_to_frame(2), force_threshold);

    float torque_threshold = 0.0001;
    EXPECT_NEAR(force_to_frame.torque(0), expected_torque_to_frame(0), torque_threshold);
    EXPECT_NEAR(force_to_frame.torque(1), expected_torque_to_frame(1), torque_threshold);
    EXPECT_NEAR(force_to_frame.torque(2), expected_torque_to_frame(2), torque_threshold);
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

    Force3D force_to_frame, force_to_edge;
    float torque_center_point_for_edge_line;

    force_calculation(edge, edge_node, pose, force_to_frame, force_to_edge, torque_center_point_for_edge_line);

    Eigen::Vector3f expected_force_to_frame = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f expected_torque_to_frame = Eigen::Vector3f(0, 0, -std::sin(M_PI/4));

    float force_threshold = 0.0001;
    EXPECT_NEAR(force_to_frame.force(0), expected_force_to_frame(0), force_threshold);
    EXPECT_NEAR(force_to_frame.force(1), expected_force_to_frame(1), force_threshold);
    EXPECT_NEAR(force_to_frame.force(2), expected_force_to_frame(2), force_threshold);

    float torque_threshold = 0.0001;
    EXPECT_NEAR(force_to_frame.torque(0), expected_torque_to_frame(0), torque_threshold);
    EXPECT_NEAR(force_to_frame.torque(1), expected_torque_to_frame(1), torque_threshold);
    // TODO: Fix this test
    // EXPECT_NEAR(force_to_frame.torque(2), expected_torque_to_frame(2), torque_threshold);
}

