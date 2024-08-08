#include <gtest/gtest.h>
#include "force_calculation.hpp"

// Helper function for comparing forces
void compareForces(const Force3D& force, const Eigen::Vector3f& expected_force, float threshold = 0.0001) {
    EXPECT_NEAR(force.force(0), expected_force(0), threshold);
    EXPECT_NEAR(force.force(1), expected_force(1), threshold);
    EXPECT_NEAR(force.force(2), expected_force(2), threshold);
}

// Helper function for comparing torques
void compareTorques(const Force3D& force, const Eigen::Vector3f& expected_torque, float threshold = 0.0001) {
    EXPECT_NEAR(force.torque(0), expected_torque(0), threshold);
    EXPECT_NEAR(force.torque(1), expected_torque(1), threshold);
    EXPECT_NEAR(force.torque(2), expected_torque(2), threshold);
}

TEST(ForceCalculationTest, force_3d_test) {
    Force3D force;
    Eigen::Vector3f expected_force = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f expected_torque = Eigen::Vector3f(0, 0, 0);

    compareForces(force, expected_force);
    compareTorques(force, expected_torque);

    Eigen::Vector3f new_force = Eigen::Vector3f(1, 2, 3);
    Eigen::Vector3f new_torque = Eigen::Vector3f(4, 5, 6);
    Force3D new_force_3d(new_force, new_torque);

    force.add(new_force_3d);
    
    expected_force = new_force;
    expected_torque = new_torque;

    compareForces(force, expected_force);
    compareTorques(force, expected_torque);
}

TEST(ForceCalculationTest, force_calculation_test) {
    Line3D edge(0, Eigen::Vector3f(0, 1, 1), Eigen::Vector3f(1, 0, 0), 1);
    EdgeNode edge_node(Eigen::Vector3f(0, 0, 1), Eigen::Vector2f(1, 0), 0);
    Pose3D pose;

    ForceCalculation force_calculation(edge, edge_node, pose);
    ASSERT_TRUE(force_calculation.calculate());

    Force3D force_to_frame = force_calculation.getForceToFrame();
    Eigen::Vector3f expected_force_to_frame = Eigen::Vector3f(0, 1, 0);
    Eigen::Vector3f expected_torque_to_frame = Eigen::Vector3f(-std::sin(M_PI/4), 0, 0);

    compareForces(force_to_frame, expected_force_to_frame);
    compareTorques(force_to_frame, expected_torque_to_frame);

    Force3D force_to_edge = force_calculation.getForceToEdge();
    Eigen::Vector3f expected_force_to_edge = -expected_force_to_frame;
    Eigen::Vector3f expected_torque_to_edge = Eigen::Vector3f(0, 0, 0);

    compareForces(force_to_edge, expected_force_to_edge);
    compareTorques(force_to_edge, expected_torque_to_edge);
}

TEST(ForceCalculationTest, force_calculation_test2) {
    Line3D edge(0, Eigen::Vector3f(1, 0, 1), Eigen::Vector3f(0, 1, 0), 1);
    EdgeNode edge_node(Eigen::Vector3f(0, 0, 1), Eigen::Vector2f(0, 1), 0);
    Pose3D pose;

    ForceCalculation force_calculation(edge, edge_node, pose);
    ASSERT_TRUE(force_calculation.calculate());

    Force3D force_to_frame = force_calculation.getForceToFrame();
    Eigen::Vector3f expected_force_to_frame = Eigen::Vector3f(1, 0, 0);
    Eigen::Vector3f expected_torque_to_frame = Eigen::Vector3f(0, std::sin(M_PI/4), 0);

    compareForces(force_to_frame, expected_force_to_frame);
    compareTorques(force_to_frame, expected_torque_to_frame);

    Force3D force_to_edge = force_calculation.getForceToEdge();
    Eigen::Vector3f expected_force_to_edge = -expected_force_to_frame;
    Eigen::Vector3f expected_torque_to_edge = Eigen::Vector3f(0, 0, 0);

    compareForces(force_to_edge, expected_force_to_edge);
    compareTorques(force_to_edge, expected_torque_to_edge);
}

TEST(ForceCalculationTest, torque_to_adjust_test_about_x_axis) {
    Line3D edge(0, Eigen::Vector3f(1, 0, 1), Eigen::Vector3f(0, 1, 0), 1);
    EdgeNode edge_node(Eigen::Vector3f(1, 0, 1), Eigen::Vector2f(0, 1), 0);
    Pose3D pose;
    pose.rotate(Eigen::Vector3f(0.3f, 0.0f, 0.0f));

    ForceCalculation force_calculation(edge, edge_node, pose);
    ASSERT_TRUE(force_calculation.calculate());

    Force3D force_to_frame = force_calculation.getForceToFrame();

    // torque(0) should be less than 0
    EXPECT_LT(force_to_frame.torque(0), 0);
}

TEST(ForceCalculationTest, torque_to_adjust_test_about_y_axis) {
    Line3D edge(0, Eigen::Vector3f(0, 1, 1), Eigen::Vector3f(1, 0, 0), 1);
    EdgeNode edge_node(Eigen::Vector3f(0, 1, 1), Eigen::Vector2f(1, 0), 0);
    Pose3D pose;
    pose.rotate(Eigen::Vector3f(0.0f, 0.3f, 0.0f));

    ForceCalculation force_calculation(edge, edge_node, pose);
    ASSERT_TRUE(force_calculation.calculate());

    Force3D force_to_frame = force_calculation.getForceToFrame();

    // torque(1) should be less than 0
    EXPECT_LT(force_to_frame.torque(1), 0);
}

TEST(ForceCalculationTest, force_should_be_same_about_x_and_y_axis) {
    Line3D edge1(0, Eigen::Vector3f(1, 0, 1), Eigen::Vector3f(0, 1, 0), 1);
    EdgeNode edge_node1(Eigen::Vector3f(1, 0, 1), Eigen::Vector2f(0, 1), 0);
    Pose3D pose;
    pose.translate(Eigen::Vector3f(0.1f, 0.0f, 0.0f));

    ForceCalculation force_calculation1(edge1, edge_node1, pose);
    ASSERT_TRUE(force_calculation1.calculate());
    Force3D force_to_frame1 = force_calculation1.getForceToFrame();

    Line3D edge2(0, Eigen::Vector3f(0, 1, 1), Eigen::Vector3f(1, 0, 0), 1);
    EdgeNode edge_node2(Eigen::Vector3f(0, 1, 1), Eigen::Vector2f(1, 0), 0);
    Pose3D pose2;
    pose2.translate(Eigen::Vector3f(0.0f, 0.1f, 0.0f));

    ForceCalculation force_calculation2(edge2, edge_node2, pose2);
    ASSERT_TRUE(force_calculation2.calculate());
    Force3D force_to_frame2 = force_calculation2.getForceToFrame();

    EXPECT_EQ(force_to_frame1.force(0), force_to_frame2.force(1));
    EXPECT_EQ(force_to_frame1.force(1), force_to_frame2.force(0));
    EXPECT_EQ(force_to_frame1.force(2), force_to_frame2.force(2));

    EXPECT_EQ(force_to_frame1.torque(0), force_to_frame2.torque(1));
    EXPECT_EQ(force_to_frame1.torque(1), -force_to_frame2.torque(0));
    EXPECT_EQ(force_to_frame1.torque(2), force_to_frame2.torque(2));
}

TEST(ForceCalculationTest, force_should_be_zero_if_translation_direction_is_same_as_edge_direction_about_x_axis) {
    Line3D edge(0, Eigen::Vector3f(0, 1, 1), Eigen::Vector3f(1, 0, 0), 1);
    EdgeNode edge_node(Eigen::Vector3f(0, 1, 1), Eigen::Vector2f(1, 0), 0);
    Pose3D pose;
    pose.translate(Eigen::Vector3f(1, 0, 0));

    ForceCalculation force_calculation(edge, edge_node, pose);
    ASSERT_TRUE(force_calculation.calculate());

    Force3D force_to_frame = force_calculation.getForceToFrame();
    Eigen::Vector3f expected_force_to_frame = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f expected_torque_to_frame = Eigen::Vector3f(0, 0, 0);

    compareForces(force_to_frame, expected_force_to_frame);
    compareTorques(force_to_frame, expected_torque_to_frame);

    Force3D force_to_edge = force_calculation.getForceToEdge();
    Eigen::Vector3f expected_force_to_edge = -expected_force_to_frame;
    Eigen::Vector3f expected_torque_to_edge = Eigen::Vector3f(0, 0, 0);

    compareForces(force_to_edge, expected_force_to_edge);
    compareTorques(force_to_edge, expected_torque_to_edge);
}

TEST(ForceCalculationTest, force_should_be_zero_if_translation_direction_is_same_as_edge_direction_about_y_axis) {
    Line3D edge(0, Eigen::Vector3f(1, 0, 1), Eigen::Vector3f(0, 1, 0), 1);
    EdgeNode edge_node(Eigen::Vector3f(1, 0, 1), Eigen::Vector2f(0, 1), 0);
    Pose3D pose;
    pose.translate(Eigen::Vector3f(0, 1, 0));

    ForceCalculation force_calculation(edge, edge_node, pose);
    ASSERT_TRUE(force_calculation.calculate());

    Force3D force_to_frame = force_calculation.getForceToFrame();
    Eigen::Vector3f expected_force_to_frame = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f expected_torque_to_frame = Eigen::Vector3f(0, 0, 0);

    compareForces(force_to_frame, expected_force_to_frame);
    compareTorques(force_to_frame, expected_torque_to_frame);

    Force3D force_to_edge = force_calculation.getForceToEdge();
    Eigen::Vector3f expected_force_to_edge = -expected_force_to_frame;
    Eigen::Vector3f expected_torque_to_edge = Eigen::Vector3f(0, 0, 0);

    compareForces(force_to_edge, expected_force_to_edge);
    compareTorques(force_to_edge, expected_torque_to_edge);
}

// Helper functions
void setupPosesAndNodes(float baseline_length, Pose3D& pose1, Pose3D& pose2, EdgeNode& edge_node1, EdgeNode& edge_node2) {
    pose1.translate(Eigen::Vector3f(0, baseline_length / 2.0f, 0));
    pose2.translate(Eigen::Vector3f(0, -baseline_length / 2.0f, 0));
    edge_node1 = EdgeNode(Eigen::Vector3f(0, -baseline_length / 2.0f, 1), Eigen::Vector2f(1, 0), 0);
    edge_node2 = EdgeNode(Eigen::Vector3f(0, baseline_length / 2.0f, 1), Eigen::Vector2f(1, 0), 0);
}

void checkForceZero(ForceCalculation& force_calculation) {
    ASSERT_TRUE(force_calculation.calculate());
    compareForces(force_calculation.getForceToEdge(), Eigen::Vector3f(0, 0, 0));
}

TEST(ForceCalculationTest, checkForceZero) {
    float baseline_length = 0.1;

    Pose3D pose1, pose2;
    EdgeNode edge_node1, edge_node2;
    setupPosesAndNodes(baseline_length, pose1, pose2, edge_node1, edge_node2);

    Line3D edge(0, Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(1, 0, 0), 0);
    ForceCalculation force_calculation1(edge, edge_node1, pose1);
    ForceCalculation force_calculation2(edge, edge_node2, pose2);

    checkForceZero(force_calculation1);
    checkForceZero(force_calculation2);
}

TEST(ForceCalculationTest, checkForceSymmetry) {
    float baseline_length = 0.1;

    Pose3D pose1, pose2;
    EdgeNode edge_node1, edge_node2;
    setupPosesAndNodes(baseline_length, pose1, pose2, edge_node1, edge_node2);

    Line3D edge(0, Eigen::Vector3f(0, 0, 0.1), Eigen::Vector3f(1, 0, 0), 1);
    ForceCalculation force_calculation1(edge, edge_node1, pose1);
    ForceCalculation force_calculation2(edge, edge_node2, pose2);

    ASSERT_TRUE(force_calculation1.calculate());
    ASSERT_TRUE(force_calculation2.calculate());

    Force3D force_to_edge1 = force_calculation1.getForceToEdge();
    Force3D force_to_edge2 = force_calculation2.getForceToEdge();

    // check the force and torque are symmetric about x-z plane
    EXPECT_EQ(force_to_edge1.force(0), force_to_edge2.force(0));
    EXPECT_EQ(force_to_edge1.force(1), -force_to_edge2.force(1));
    EXPECT_EQ(force_to_edge1.force(2), force_to_edge2.force(2));

    EXPECT_EQ(force_to_edge1.torque(0), force_to_edge2.torque(0));
    EXPECT_EQ(force_to_edge1.torque(1), -force_to_edge2.torque(1));
    EXPECT_EQ(force_to_edge1.torque(2), force_to_edge2.torque(2));

    EXPECT_EQ(force_to_edge1.force(0), 0);
    EXPECT_GT(force_to_edge1.force(2), 0);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
