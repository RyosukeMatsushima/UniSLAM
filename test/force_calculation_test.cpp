#include <gtest/gtest.h>

#include "force_calculation.hpp"

TEST(ForceCalculationTest, force_3d_test) {
    Force3D force;
    Eigen::Vector3f expected_force = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f expected_torque = Eigen::Vector3f(0, 0, 0);

    float force_threshold = 0.0001;
    EXPECT_NEAR(force.force(0), expected_force(0), force_threshold);
    EXPECT_NEAR(force.force(1), expected_force(1), force_threshold);
    EXPECT_NEAR(force.force(2), expected_force(2), force_threshold);

    float torque_threshold = 0.0001;
    EXPECT_NEAR(force.torque(0), expected_torque(0), torque_threshold);
    EXPECT_NEAR(force.torque(1), expected_torque(1), torque_threshold);
    EXPECT_NEAR(force.torque(2), expected_torque(2), torque_threshold);

    Eigen::Vector3f new_force = Eigen::Vector3f(1, 2, 3);
    Eigen::Vector3f new_torque = Eigen::Vector3f(4, 5, 6);
    Force3D new_force_3d(new_force, new_torque);

    force.add(new_force_3d);
    
    expected_force = new_force;
    expected_torque = new_torque;

    EXPECT_NEAR(force.force(0), expected_force(0), force_threshold);
    EXPECT_NEAR(force.force(1), expected_force(1), force_threshold);
    EXPECT_NEAR(force.force(2), expected_force(2), force_threshold);

    EXPECT_NEAR(force.torque(0), expected_torque(0), torque_threshold);
    EXPECT_NEAR(force.torque(1), expected_torque(1), torque_threshold);
    EXPECT_NEAR(force.torque(2), expected_torque(2), torque_threshold);
}

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

    Eigen::Vector3f expected_force_to_edge = -expected_force_to_frame;
    EXPECT_NEAR(force_to_edge.force(0), expected_force_to_edge(0), force_threshold);
    EXPECT_NEAR(force_to_edge.force(1), expected_force_to_edge(1), force_threshold);
    EXPECT_NEAR(force_to_edge.force(2), expected_force_to_edge(2), force_threshold);

    Eigen::Vector3f expected_torque_to_edge = Eigen::Vector3f(0, 0, 0);
    EXPECT_NEAR(force_to_edge.torque(0), expected_torque_to_edge(0), torque_threshold);
    EXPECT_NEAR(force_to_edge.torque(1), expected_torque_to_edge(1), torque_threshold);
    EXPECT_NEAR(force_to_edge.torque(2), expected_torque_to_edge(2), torque_threshold);
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
    EXPECT_NEAR(force_to_frame.torque(2), expected_torque_to_frame(2), torque_threshold);

    Eigen::Vector3f expected_force_to_edge = -expected_force_to_frame;
    EXPECT_NEAR(force_to_edge.force(0), expected_force_to_edge(0), force_threshold);
    EXPECT_NEAR(force_to_edge.force(1), expected_force_to_edge(1), force_threshold);
    EXPECT_NEAR(force_to_edge.force(2), expected_force_to_edge(2), force_threshold);

    Eigen::Vector3f expected_torque_to_edge = Eigen::Vector3f(0, 0, std::sin(M_PI/4));
    EXPECT_NEAR(force_to_edge.torque(0), expected_torque_to_edge(0), torque_threshold);
    EXPECT_NEAR(force_to_edge.torque(1), expected_torque_to_edge(1), torque_threshold);
    EXPECT_NEAR(force_to_edge.torque(2), expected_torque_to_edge(2), torque_threshold);
}

TEST(ForceCalculationTest, force_and_torque_calculation_test) {

    Line3D edge(0,
                Eigen::Vector3f(0, 1, 1),
                Eigen::Vector3f(1, 0, 0),
                1);

    EdgeNode edge_node(Eigen::Vector3f(0, 0, 1),
                       Eigen::Vector2f(1, 1),
                       0);

    Pose3D pose;

    Force3D force_to_frame, force_to_edge;
    float torque_center_point_for_edge_line;

    force_calculation(edge, edge_node, pose, force_to_frame, force_to_edge, torque_center_point_for_edge_line);

    Eigen::Vector3f expected_force_to_frame = Eigen::Vector3f(0, 1, 0);
    Eigen::Vector3f expected_torque_to_frame = Eigen::Vector3f(-std::sin(M_PI/4), 0, -std::sin(M_PI/4));

    float force_threshold = 0.0001;
    EXPECT_NEAR(force_to_frame.force(0), expected_force_to_frame(0), force_threshold);
    EXPECT_NEAR(force_to_frame.force(1), expected_force_to_frame(1), force_threshold);
    EXPECT_NEAR(force_to_frame.force(2), expected_force_to_frame(2), force_threshold);

    float torque_threshold = 0.0001;
    EXPECT_NEAR(force_to_frame.torque(0), expected_torque_to_frame(0), torque_threshold);
    EXPECT_NEAR(force_to_frame.torque(1), expected_torque_to_frame(1), torque_threshold);
    EXPECT_NEAR(force_to_frame.torque(2), expected_torque_to_frame(2), torque_threshold);

    Eigen::Vector3f expected_force_to_edge = -expected_force_to_frame;
    EXPECT_NEAR(force_to_edge.force(0), expected_force_to_edge(0), force_threshold);
    EXPECT_NEAR(force_to_edge.force(1), expected_force_to_edge(1), force_threshold);
    EXPECT_NEAR(force_to_edge.force(2), expected_force_to_edge(2), force_threshold);

    Eigen::Vector3f expected_torque_to_edge = Eigen::Vector3f(0, 0, std::sin(M_PI/4));
    EXPECT_NEAR(force_to_edge.torque(0), expected_torque_to_edge(0), torque_threshold);
    EXPECT_NEAR(force_to_edge.torque(1), expected_torque_to_edge(1), torque_threshold);
    EXPECT_NEAR(force_to_edge.torque(2), expected_torque_to_edge(2), torque_threshold);
}


TEST(ForceCalculationTest, torque_calculation_with_virtical_edge_test) {

    Line3D edge(0,
                Eigen::Vector3f(0, 1, 0),
                Eigen::Vector3f(1, 0, 0),
                1);

    EdgeNode edge_node(Eigen::Vector3f(0, 1, 1),
                       Eigen::Vector2f(1, 0),
                       0);

    Pose3D pose;
    pose.translate(Eigen::Vector3f(0.0f, 0.0f, -1.0f));
    const float pose_x_rotate = 0.3f;
    pose.rotate(Eigen::Vector3f(pose_x_rotate, 0.0f, 0.0f));

    Force3D force_to_frame, force_to_edge;
    float torque_center_point_for_edge_line;

    force_calculation(edge, edge_node, pose, force_to_frame, force_to_edge, torque_center_point_for_edge_line);

    Eigen::Vector3f expected_torque_to_frame = Eigen::Vector3f(-std::sin(pose_x_rotate), 0, 0);

    float torque_threshold = 0.0001;
    EXPECT_NEAR(force_to_frame.torque(0), expected_torque_to_frame(0), torque_threshold);
    EXPECT_NEAR(force_to_frame.torque(1), expected_torque_to_frame(1), torque_threshold);
    EXPECT_NEAR(force_to_frame.torque(2), expected_torque_to_frame(2), torque_threshold);

    Eigen::Vector3f expected_torque_to_edge = Eigen::Vector3f(0, 0, 0);
    EXPECT_NEAR(force_to_edge.torque(0), expected_torque_to_edge(0), torque_threshold);
    EXPECT_NEAR(force_to_edge.torque(1), expected_torque_to_edge(1), torque_threshold);
    EXPECT_NEAR(force_to_edge.torque(2), expected_torque_to_edge(2), torque_threshold);
}

// with horizontal edge, torque should be zero
TEST(ForceCalculationTest, torque_calculation_with_horizontal_edge_test) {

    Line3D edge1(0,
                Eigen::Vector3f(1, 0, 0),
                Eigen::Vector3f(0, 1, 0),
                1);

    EdgeNode edge_node1(Eigen::Vector3f(1, 0, 1),
                       Eigen::Vector2f(0, 1),
                       0);

    Pose3D pose;
    pose.translate(Eigen::Vector3f(0.0f, 0.0f, -1.0f));
    const float pose_x_rotate = 0.3f;
    pose.rotate(Eigen::Vector3f(pose_x_rotate, 0.0f, 0.0f));

    Force3D force_to_frame1, force_to_edge1;
    float torque_center_point_for_edge_line1;

    force_calculation(edge1, edge_node1, pose, force_to_frame1, force_to_edge1, torque_center_point_for_edge_line1);

    Line3D edge2(0,
                Eigen::Vector3f(-1, 0, 0),
                Eigen::Vector3f(0, 1, 0),
                1);

    EdgeNode edge_node2(Eigen::Vector3f(-1, 0, 1),
                       Eigen::Vector2f(0, 1),
                       0);

    Force3D force_to_frame2, force_to_edge2;
    float torque_center_point_for_edge_line2;

    force_calculation(edge2, edge_node2, pose, force_to_frame2, force_to_edge2, torque_center_point_for_edge_line2);


    float force_threshold = 0.0001;
    EXPECT_NEAR(force_to_frame1.force(0), -force_to_frame2.force(0), force_threshold);
    EXPECT_NEAR(force_to_frame1.force(1), -force_to_frame2.force(1), force_threshold);
    EXPECT_NEAR(force_to_frame1.force(2), force_to_frame2.force(2), force_threshold);

    float torque_threshold = 0.0001;
    EXPECT_NEAR(force_to_frame1.torque(0), force_to_frame2.torque(0), torque_threshold);
    EXPECT_NEAR(force_to_frame1.torque(1), -force_to_frame2.torque(1), torque_threshold);
    EXPECT_NEAR(force_to_frame1.torque(2), -force_to_frame2.torque(2), torque_threshold);
}

