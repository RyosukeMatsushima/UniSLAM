#include <gtest/gtest.h>
#include "edge_space_dynamics.hpp"

#define CONFIG_YAML_PATH PROJECT_SOURCE_DIR "/config_for_test.yaml"

TEST(AddNewEdgeTest, addVirticalNewEdge) {

    Pose3D frame1_pose;
    Pose3D frame2_pose;

    frame1_pose.translate(Eigen::Vector3f(0, 0, 0));
    frame2_pose.translate(Eigen::Vector3f(1, 0, 0));

    EdgeNode frame1_edge_node(Eigen::Vector3f(1, 0, 1),
                              Eigen::Vector2f(0, 1),
                              -1);

    EdgeNode frame2_edge_node(Eigen::Vector3f(0, 0, 1),
                              Eigen::Vector2f(0, 1),
                              -1);

    EdgeSpaceDynamics edge_space_dynamics(CONFIG_YAML_PATH);

    int edge_id;

    bool result = edge_space_dynamics.add_new_edge(frame1_pose,
                                                   frame2_pose,
                                                   frame1_edge_node,
                                                   frame2_edge_node,
                                                   edge_id);

    // edge_id should be 0
    EXPECT_EQ(edge_id, 0);

    // result should be true
    EXPECT_EQ(result, true);

    // check added edge
    Line3D expected_edge(0,
                         Eigen::Vector3f(1, 0, 1),
                         Eigen::Vector3f(0, 1, 0),
                         0);

    std::vector<Line3D> edges = edge_space_dynamics.get_edge3ds();

    float threshold = 0.001;
    ASSERT_EQ(edges.size(), 1);
    EXPECT_NEAR(edges[0].start_point()[0], expected_edge.start_point()[0], threshold);
    EXPECT_NEAR(edges[0].start_point()[1], expected_edge.start_point()[1], threshold);
    EXPECT_NEAR(edges[0].start_point()[2], expected_edge.start_point()[2], threshold);
    EXPECT_NEAR(edges[0].direction()[0], expected_edge.direction()[0], threshold);
    EXPECT_NEAR(edges[0].direction()[1], expected_edge.direction()[1], threshold);
    EXPECT_NEAR(edges[0].direction()[2], expected_edge.direction()[2], threshold);
}

TEST(AddNewEdgeTest, addHorizontalNewEdge) {

    Line3D expected_edge(0,
                         Eigen::Vector3f(0, 1, 1),
                         Eigen::Vector3f(1, 0, 0),
                         0);


    Pose3D frame1_pose;
    Pose3D frame2_pose;

    frame1_pose.translate(Eigen::Vector3f(0, 0, 0));
    frame2_pose.translate(Eigen::Vector3f(0, 1, 0));

    EdgeNode frame1_edge_node(Eigen::Vector3f(0, 1, 1),
                              Eigen::Vector2f(1, 0),
                              -1);

    EdgeNode frame2_edge_node(Eigen::Vector3f(0, 0, 1),
                              Eigen::Vector2f(1, 0),
                              -1);

    EdgeSpaceDynamics edge_space_dynamics(CONFIG_YAML_PATH);

    int edge_id;

    bool result = edge_space_dynamics.add_new_edge(frame1_pose,
                                                   frame2_pose,
                                                   frame1_edge_node,
                                                   frame2_edge_node,
                                                   edge_id);

    // edge_id should be 0
    EXPECT_EQ(edge_id, 0);

    // result should be true
    EXPECT_EQ(result, true);

    // check added edge
    std::vector<Line3D> edges = edge_space_dynamics.get_edge3ds();

    float threshold = 0.001;
    ASSERT_EQ(edges.size(), 1);
    EXPECT_NEAR(edges[0].start_point()[0], expected_edge.start_point()[0], threshold);
    EXPECT_NEAR(edges[0].start_point()[1], expected_edge.start_point()[1], threshold);
    EXPECT_NEAR(edges[0].start_point()[2], expected_edge.start_point()[2], threshold);
    EXPECT_NEAR(edges[0].direction()[0], expected_edge.direction()[0], threshold);
    EXPECT_NEAR(edges[0].direction()[1], expected_edge.direction()[1], threshold);
    EXPECT_NEAR(edges[0].direction()[2], expected_edge.direction()[2], threshold);
}

TEST(AddNewEdgeTest, addZAxisNewEdge) {

    Line3D expected_edge(0,
                         Eigen::Vector3f(0, 0, 1.4),
                         Eigen::Vector3f(0, 0, 1),
                         0);

    Pose3D frame1_pose;
    Pose3D frame2_pose;

    frame1_pose.translate(Eigen::Vector3f(-1, 1, 0));
    frame2_pose.translate(Eigen::Vector3f(1, 1, 0));

    EdgeNode frame1_edge_node(Eigen::Vector3f(1, -1, 1),
                              Eigen::Vector2f(-1, -1),
                              -1);

    EdgeNode frame2_edge_node(Eigen::Vector3f(-1, -1, 1),
                              Eigen::Vector2f(1, -1),
                              -1);

    EdgeSpaceDynamics edge_space_dynamics(CONFIG_YAML_PATH);

    int edge_id;

    bool result = edge_space_dynamics.add_new_edge(frame1_pose,
                                                   frame2_pose,
                                                   frame1_edge_node,
                                                   frame2_edge_node,
                                                   edge_id);

    // edge_id should be 0
    EXPECT_EQ(edge_id, 0);

    // result should be true
    EXPECT_EQ(result, true);

    // check added edge
    std::vector<Line3D> edges = edge_space_dynamics.get_edge3ds();
    ASSERT_EQ(edges.size(), 1);

    Line3D edge = edges[0];

    EXPECT_TRUE(expected_edge.connect(edge));

    float threshold = 0.001;
    EXPECT_NEAR(edges[0].start_point()[0], expected_edge.start_point()[0], threshold);
    EXPECT_NEAR(edges[0].start_point()[1], expected_edge.start_point()[1], threshold);

    threshold = 0.2;
    EXPECT_NEAR(edges[0].direction()[0], expected_edge.direction()[0], threshold);
    EXPECT_NEAR(edges[0].direction()[1], expected_edge.direction()[1], threshold);
    EXPECT_NEAR(edges[0].direction()[2], expected_edge.direction()[2], threshold);
}

// add new edge with same pose
// should return false
TEST(AddNewEdgeTest, addEdgeWithSamePose) {

    Pose3D frame1_pose;
    Pose3D frame2_pose;

    frame1_pose.translate(Eigen::Vector3f(1, 0, 0));
    frame2_pose.translate(Eigen::Vector3f(1, 0, 0));

    EdgeNode frame1_edge_node(Eigen::Vector3f(-1, 0, 1),
                              Eigen::Vector2f(0, 1),
                              -1);

    EdgeNode frame2_edge_node(Eigen::Vector3f(-1, 0, 1),
                              Eigen::Vector2f(0, 1),
                              -1);

    EdgeSpaceDynamics edge_space_dynamics(CONFIG_YAML_PATH);

    int edge_id;

    bool result = edge_space_dynamics.add_new_edge(frame1_pose,
                                                   frame2_pose,
                                                   frame1_edge_node,
                                                   frame2_edge_node,
                                                   edge_id);

    // edge_id should be 0
    EXPECT_EQ(edge_id, 0);

    // result should be false
    EXPECT_FALSE(result);
}

// add new edge in the case the direction of the edge and pose1 to pose2 is same
// should return false
TEST(AddNewEdgeTest, addEdgeWithSameDirection) {

    Pose3D frame1_pose;
    Pose3D frame2_pose;

    frame1_pose.translate(Eigen::Vector3f(1, 0, 0));
    frame2_pose.translate(Eigen::Vector3f(-1, 0, 0));

    EdgeNode frame1_edge_node(Eigen::Vector3f(-1, 0, 1),
                              Eigen::Vector2f(1, 0),
                              -1);

    EdgeNode frame2_edge_node(Eigen::Vector3f(1, 0, 1),
                              Eigen::Vector2f(1, 0),
                              -1);

    EdgeSpaceDynamics edge_space_dynamics(CONFIG_YAML_PATH);

    int edge_id;

    bool result = edge_space_dynamics.add_new_edge(frame1_pose,
                                                   frame2_pose,
                                                   frame1_edge_node,
                                                   frame2_edge_node,
                                                   edge_id);

    // edge_id should be 0
    EXPECT_EQ(edge_id, 0);

    // result should be false
    EXPECT_FALSE(result);
}


// add new edge in the case pose1 to pose2 is close
// should return true
TEST(AddNewEdgeTest, addEdgeWithCloseFrames) {

    Line3D expected_edge(0,
                         Eigen::Vector3f(0, 0, 1),
                         Eigen::Vector3f(1, 0, 0),
                         0);

    Pose3D frame1_pose;
    Pose3D frame2_pose;

    float baseline = 0.1;

    frame1_pose.translate(Eigen::Vector3f(0, -baseline / 2.0f, 0));
    frame2_pose.translate(Eigen::Vector3f(0, baseline / 2.0f, 0));

    EdgeNode frame1_edge_node(Eigen::Vector3f(0, baseline / 2.0f, 1),
                              Eigen::Vector2f(1, 0),
                              -1);

    EdgeNode frame2_edge_node(Eigen::Vector3f(0, -baseline / 2.0f, 1),
                              Eigen::Vector2f(1, 0),
                              -1);

    EdgeSpaceDynamics edge_space_dynamics(CONFIG_YAML_PATH);

    int edge_id;

    bool result = edge_space_dynamics.add_new_edge(frame1_pose,
                                                   frame2_pose,
                                                   frame1_edge_node,
                                                   frame2_edge_node,
                                                   edge_id);

    // edge_id should be 0
    EXPECT_EQ(edge_id, 0);

    // result should be false
    ASSERT_TRUE(result);

    // check added edge
    Line3D edge = edge_space_dynamics.get_edge3ds()[0];

    float threshold = 0.01;
    EXPECT_NEAR(edge.start_point()[0], expected_edge.start_point()[0], threshold);
    EXPECT_NEAR(edge.start_point()[1], expected_edge.start_point()[1], threshold);
    EXPECT_NEAR(edge.start_point()[2], expected_edge.start_point()[2], threshold);

    EXPECT_NEAR(edge.direction()[0], expected_edge.direction()[0], threshold);
    EXPECT_NEAR(edge.direction()[1], expected_edge.direction()[1], threshold);
    EXPECT_NEAR(edge.direction()[2], expected_edge.direction()[2], threshold);

}

TEST(AddNewEdgeTest, addEdgeWithCloseFramesAsymmetrical) {

    Line3D expected_edge(0,
                         Eigen::Vector3f(0, -0.5, 1),
                         Eigen::Vector3f(1, 0, 0),
                         0);

    Pose3D frame1_pose;
    Pose3D frame2_pose;

    float baseline = 0.05;

    frame1_pose.translate(Eigen::Vector3f(0, 0, 0));
    frame2_pose.translate(Eigen::Vector3f(0, baseline, 0));

    EdgeNode frame1_edge_node(Eigen::Vector3f(0, -0.5, 1),
                              Eigen::Vector2f(1, 0),
                              -1);

    EdgeNode frame2_edge_node(Eigen::Vector3f(0, -0.5 - baseline, 1),
                              Eigen::Vector2f(1, 0),
                              -1);

    EdgeSpaceDynamics edge_space_dynamics(CONFIG_YAML_PATH);

    int edge_id;

    bool result = edge_space_dynamics.add_new_edge(frame1_pose,
                                                   frame2_pose,
                                                   frame1_edge_node,
                                                   frame2_edge_node,
                                                   edge_id);

    // edge_id should be 0
    EXPECT_EQ(edge_id, 0);

    // result should be false
    EXPECT_TRUE(result);

    // check added edge
    ASSERT_EQ(edge_space_dynamics.get_edge3ds().size(), 1);
    Line3D edge = edge_space_dynamics.get_edge3ds()[0];

    EXPECT_TRUE(expected_edge.connect(edge));

    float threshold = 0.01;
    EXPECT_NEAR(edge.start_point()[1], expected_edge.start_point()[1], threshold);
    EXPECT_NEAR(edge.start_point()[2], expected_edge.start_point()[2], threshold);

    EXPECT_NEAR(edge.direction()[0], expected_edge.direction()[0], threshold);
    EXPECT_NEAR(edge.direction()[1], expected_edge.direction()[1], threshold);
    EXPECT_NEAR(edge.direction()[2], expected_edge.direction()[2], threshold);

}

