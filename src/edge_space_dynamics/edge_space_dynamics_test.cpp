#include <gtest/gtest.h>

#include "edge_space_dynamics.hpp"

TEST(EdgeSpaceDynamics, calculateFramePose) {

    EdgeSpaceDynamics edge_space_dynamics;

    // set the edges
    int edge1_id = edge_space_dynamics.set_edge3d(Eigen::Vector3f(1.0, 0.0, 0.0), // start_point
                                                  Eigen::Vector3f(0.0, 1.0, 0.0), // direction
                                                  1.0); // length

    int edge2_id = edge_space_dynamics.set_edge3d(Eigen::Vector3f(-1.0, 0.0, 0.0),
                                                  Eigen::Vector3f(0.0, 1.0, 0.0),
                                                  1.0);

    int edge3_id = edge_space_dynamics.set_edge3d(Eigen::Vector3f(0.0, 1.0, 0.0),
                                                  Eigen::Vector3f(1.0, 0.0, 0.0),
                                                  1.0);

    int edge4_id = edge_space_dynamics.set_edge3d(Eigen::Vector3f(0.0, -1.0, 0.0),
                                                  Eigen::Vector3f(1.0, 0.0, 0.0),
                                                  1.0);


    // EdgeNode represents the detected edge in the frame coordinate
    EdgeNode edge_node1 = EdgeNode(Eigen::Vector3f(1.0, 0.0, 1.0), // direction_frame_to_edge
                                   Eigen::Vector2f(0.0, 1.0), // edge_direction
                                   edge1_id); // edge_id

    EdgeNode edge_node2 = EdgeNode(Eigen::Vector3f(-1.0, 0.0, 1.0),
                                   Eigen::Vector2f(0.0, 1.0),
                                   edge2_id);

    EdgeNode edge_node3 = EdgeNode(Eigen::Vector3f(0.0, 1.0, 1.0),
                                   Eigen::Vector2f(1.0, 0.0),
                                   edge3_id);

    EdgeNode edge_node4 = EdgeNode(Eigen::Vector3f(0.0, -1.0, 1.0),
                                   Eigen::Vector2f(1.0, 0.0),
                                   edge4_id);

    std::vector<EdgeNode> edge_nodes = {edge_node1, edge_node2, edge_node3, edge_node4};


    // user can set initial frame pose for calculation
    Pose3D frame_pose;

    // move to random pose
    frame_pose.translate(Eigen::Vector3f(0.0f, 0.0f, -1.2f));
    frame_pose.rotate(Eigen::Vector3f(0.1f, 0.0f, 0.0f));

    bool result = edge_space_dynamics.calculate_frame_pose(edge_nodes, frame_pose);

    Eigen::Vector3f expected_position = Eigen::Vector3f(0.0, 0.0, -1.0);
    Eigen::Quaternionf expected_orientation = Eigen::Quaternionf::Identity();

    // check the result
    EXPECT_TRUE(result);

    // check the position
    float position_error_threshold = 1e-4;
    EXPECT_NEAR(frame_pose.position.x(), expected_position.x(), position_error_threshold);
    EXPECT_NEAR(frame_pose.position.y(), expected_position.y(), position_error_threshold);
    EXPECT_NEAR(frame_pose.position.z(), expected_position.z(), position_error_threshold);

    // check the orientation
    float rotate_error_threshold = 1e-4;
    EXPECT_NEAR(frame_pose.orientation.x(), expected_orientation.x(), rotate_error_threshold);
    EXPECT_NEAR(frame_pose.orientation.y(), expected_orientation.y(), rotate_error_threshold);
    EXPECT_NEAR(frame_pose.orientation.z(), expected_orientation.z(), rotate_error_threshold);
    EXPECT_NEAR(frame_pose.orientation.w(), expected_orientation.w(), rotate_error_threshold);
}

TEST(EdgeSpaceDynamics, calculateFramePoseHolizontalMoveTest) {

    EdgeSpaceDynamics edge_space_dynamics;

    // set the edges
    int edge1_id = edge_space_dynamics.set_edge3d(Eigen::Vector3f(1.0, 0.0, 0.0), // start_point
                                                  Eigen::Vector3f(0.0, 1.0, 0.0), // direction
                                                  1.0); // length

    int edge2_id = edge_space_dynamics.set_edge3d(Eigen::Vector3f(-1.0, 0.0, 0.0),
                                                  Eigen::Vector3f(0.0, 1.0, 0.0),
                                                  1.0);

    int edge3_id = edge_space_dynamics.set_edge3d(Eigen::Vector3f(0.0, 1.0, 0.0),
                                                  Eigen::Vector3f(1.0, 0.0, 0.0),
                                                  1.0);

    int edge4_id = edge_space_dynamics.set_edge3d(Eigen::Vector3f(0.0, -1.0, 0.0),
                                                  Eigen::Vector3f(1.0, 0.0, 0.0),
                                                  1.0);


    // EdgeNode represents the detected edge in the frame coordinate
    EdgeNode edge_node1 = EdgeNode(Eigen::Vector3f(1.0, 0.0, 1.0), // direction_frame_to_edge
                                   Eigen::Vector2f(0.0, 1.0), // edge_direction
                                   edge1_id); // edge_id

    EdgeNode edge_node2 = EdgeNode(Eigen::Vector3f(-1.0, 0.0, 1.0),
                                   Eigen::Vector2f(0.0, 1.0),
                                   edge2_id);

    EdgeNode edge_node3 = EdgeNode(Eigen::Vector3f(0.0, 1.0, 1.0),
                                   Eigen::Vector2f(1.0, 0.0),
                                   edge3_id);

    EdgeNode edge_node4 = EdgeNode(Eigen::Vector3f(0.0, -1.0, 1.0),
                                   Eigen::Vector2f(1.0, 0.0),
                                   edge4_id);

    std::vector<EdgeNode> edge_nodes = {edge_node1, edge_node2, edge_node3, edge_node4};


    // user can set initial frame pose for calculation
    Pose3D frame_pose;

    // move to random pose
    frame_pose.translate(Eigen::Vector3f(0.1f, 1.0f, -1.2f));
    frame_pose.rotate(Eigen::Vector3f(1.0f, -2.0f, 1.1f));

    bool result = edge_space_dynamics.calculate_frame_pose(edge_nodes, frame_pose);

    Eigen::Vector3f expected_position = Eigen::Vector3f(0.0, 0.0, -1.0);
    Eigen::Quaternionf expected_orientation = Eigen::Quaternionf::Identity();

    // check the result
    EXPECT_TRUE(result);

    // check the position
    float position_error_threshold = 1e-4;
    EXPECT_NEAR(frame_pose.position.x(), expected_position.x(), position_error_threshold);
    EXPECT_NEAR(frame_pose.position.y(), expected_position.y(), position_error_threshold);
    EXPECT_NEAR(frame_pose.position.z(), expected_position.z(), position_error_threshold);

    // check the orientation
    float rotate_error_threshold = 1e-4;
    EXPECT_NEAR(frame_pose.orientation.coeffs().x(), expected_orientation.coeffs().x(), rotate_error_threshold);
    EXPECT_NEAR(frame_pose.orientation.coeffs().y(), expected_orientation.coeffs().y(), rotate_error_threshold);
    EXPECT_NEAR(frame_pose.orientation.coeffs().z(), expected_orientation.coeffs().z(), rotate_error_threshold);
    EXPECT_NEAR(frame_pose.orientation.coeffs().w(), expected_orientation.coeffs().w(), rotate_error_threshold);
}

