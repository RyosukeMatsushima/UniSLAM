#include <gtest/gtest.h>
#include "edge_space_dynamics.hpp"

TEST(AddNewEdgeTest, addVirticalNewEdge) {

    Pose3D frame1_pose;
    Pose3D frame2_pose;
    
    frame1_pose.translate(Eigen::Vector3f(1, 0, 0));
    frame2_pose.translate(Eigen::Vector3f(-1, 0, 0));

    EdgeNode frame1_edge_node(Eigen::Vector3f(-1, 0, 1),
                              Eigen::Vector2f(0, 1),
                              0);

    EdgeNode frame2_edge_node(Eigen::Vector3f(1, 0, 1),
                              Eigen::Vector2f(0, 1),
                              1);

    EdgeSpaceDynamics edge_space_dynamics;

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
                         Eigen::Vector3f(0, 0, 1),
                         Eigen::Vector3f(0, 1, 0),
                         0);

    std::vector<Line3D> edges = edge_space_dynamics.get_edge3ds();

    float threshold = 0.0001;
    EXPECT_EQ(edges.size(), 1);
    EXPECT_NEAR(edges[0].start_point()[0], expected_edge.start_point()[0], threshold);
    EXPECT_NEAR(edges[0].start_point()[1], expected_edge.start_point()[1], threshold);
    EXPECT_NEAR(edges[0].start_point()[2], expected_edge.start_point()[2], threshold);
    EXPECT_NEAR(edges[0].direction()[0], expected_edge.direction()[0], threshold);
    EXPECT_NEAR(edges[0].direction()[1], expected_edge.direction()[1], threshold);
    EXPECT_NEAR(edges[0].direction()[2], expected_edge.direction()[2], threshold);
}
