#include <gtest/gtest.h>
#include "edge_space_dynamics.hpp"

#define CONFIG_YAML_PATH PROJECT_SOURCE_DIR "/config_for_test.yaml"

class FrameData {
public:
    Pose3D pose;
    const Pose3D correct_pose;
    std::vector<EdgeNode> edge_nodes;

    FrameData(const Pose3D& pose, const std::vector<EdgeNode>& edge_nodes)
        : pose(pose), correct_pose(pose), edge_nodes(edge_nodes) {}

    void setPose(const Pose3D& pose) {
        this->pose = pose;
    }
};

class EdgeData {
public:
    Eigen::Vector3f start_point;
    Eigen::Vector3f direction;
    float length;
    int id;

    const Eigen::Vector3f correct_start_point;
    const Eigen::Vector3f correct_direction;
    const float correct_length;

    EdgeData(const Eigen::Vector3f& start_point, const Eigen::Vector3f& direction, float length)
        : start_point(start_point), direction(direction), length(length),
          correct_start_point(start_point), correct_direction(direction), correct_length(length) {}
};

class OptimizeTest : public ::testing::Test {
protected:
    EdgeSpaceDynamics edge_space_dynamics;

    FrameData frame0;
    FrameData frame1;
    FrameData frame2;
    FrameData frame3;

    EdgeData edge0;
    EdgeData edge1;
    EdgeData edge2;
    EdgeData edge3;

    OptimizeTest()
        : edge_space_dynamics(CONFIG_YAML_PATH),
          frame0(Pose3D(), {
            EdgeNode(Eigen::Vector3f(1.0, 0.0, 1.0), Eigen::Vector2f(0.0, 1.0), 0),
            EdgeNode(Eigen::Vector3f(-1.0, 0.0, 1.0), Eigen::Vector2f(0.0, 1.0), 1),
            EdgeNode(Eigen::Vector3f(0.0, 1.0, 1.0), Eigen::Vector2f(1.0, 0.0), 2),
            EdgeNode(Eigen::Vector3f(0.0, -1.0, 1.0), Eigen::Vector2f(1.0, 0.0), 3)
          }),
          frame1(Pose3D(Eigen::Vector3f(1.0, 0.0, 0.0), Eigen::Quaternionf::Identity()), {
            EdgeNode(Eigen::Vector3f(0.0, 0.0, 1.0), Eigen::Vector2f(0.0, 1.0), 0),
            EdgeNode(Eigen::Vector3f(-2.0, 0.0, 1.0), Eigen::Vector2f(0.0, 1.0), 1),
            EdgeNode(Eigen::Vector3f(0.0, 1.0, 1.0), Eigen::Vector2f(1.0, 0.0), 2),
            EdgeNode(Eigen::Vector3f(0.0, -1.0, 1.0), Eigen::Vector2f(1.0, 0.0), 3)
          }),
          frame2(Pose3D(Eigen::Vector3f(-1.0, 0.0, 0.0), Eigen::Quaternionf::Identity()), {
            EdgeNode(Eigen::Vector3f(2.0, 0.0, 1.0), Eigen::Vector2f(0.0, 1.0), 0),
            EdgeNode(Eigen::Vector3f(0.0, 0.0, 1.0), Eigen::Vector2f(0.0, 1.0), 1),
            EdgeNode(Eigen::Vector3f(0.0, 1.0, 1.0), Eigen::Vector2f(1.0, 0.0), 2),
            EdgeNode(Eigen::Vector3f(0.0, -1.0, 1.0), Eigen::Vector2f(1.0, 0.0), 3)
          }),
          frame3(Pose3D(Eigen::Vector3f(0.0, 1.0, 0.0), Eigen::Quaternionf::Identity()), {
            EdgeNode(Eigen::Vector3f(1.0, 0.0, 1.0), Eigen::Vector2f(0.0, 1.0), 0),
            EdgeNode(Eigen::Vector3f(-1.0, 0.0, 1.0), Eigen::Vector2f(0.0, 1.0), 1),
            EdgeNode(Eigen::Vector3f(0.0, 0.0, 1.0), Eigen::Vector2f(1.0, 0.0), 2),
            EdgeNode(Eigen::Vector3f(0.0, -2.0, 1.0), Eigen::Vector2f(1.0, 0.0), 3)
          }),
          edge0(Eigen::Vector3f(1.0, 0.0, 1.0), Eigen::Vector3f(0.0, 1.0, 0.0), 1.0),
          edge1(Eigen::Vector3f(-1.0, 0.0, 1.0), Eigen::Vector3f(0.0, 1.0, 0.0), 1.0),
          edge2(Eigen::Vector3f(0.0, 1.0, 1.0), Eigen::Vector3f(1.0, 0.0, 0.0), 1.0),
          edge3(Eigen::Vector3f(0.0, -1.0, 1.0), Eigen::Vector3f(1.0, 0.0, 0.0), 1.0) {}

    void setEdges() {
        edge0.id = edge_space_dynamics.set_edge3d(edge0.start_point, edge0.direction, edge0.length);
        edge1.id = edge_space_dynamics.set_edge3d(edge1.start_point, edge1.direction, edge1.length);
        edge2.id = edge_space_dynamics.set_edge3d(edge2.start_point, edge2.direction, edge2.length);
        edge3.id = edge_space_dynamics.set_edge3d(edge3.start_point, edge3.direction, edge3.length);
    }

    int addInvalidEdge() {
        EdgeData invalid_edge(Eigen::Vector3f(0.1, -1.0, 2.0), Eigen::Vector3f(-0.2, 0.1, -10.0), 1.0);
        invalid_edge.id = edge_space_dynamics.set_edge3d(invalid_edge.start_point, invalid_edge.direction, invalid_edge.length);

        // TODO: add EdgeNode with random values
        frame0.edge_nodes.push_back(EdgeNode(Eigen::Vector3f(0.1, -1.0, 2.0), Eigen::Vector2f(0.0, 1.0), invalid_edge.id));
        frame1.edge_nodes.push_back(EdgeNode(Eigen::Vector3f(0.1, -1.0, 2.0), Eigen::Vector2f(0.0, 1.0), invalid_edge.id));
        frame2.edge_nodes.push_back(EdgeNode(Eigen::Vector3f(0.1, -1.0, 2.0), Eigen::Vector2f(0.0, 1.0), invalid_edge.id));
        frame3.edge_nodes.push_back(EdgeNode(Eigen::Vector3f(0.1, -1.0, 2.0), Eigen::Vector2f(0.0, 1.0), invalid_edge.id));

        return invalid_edge.id;
    }

    // add duplicate edge with edge0
    int addDuplicateEdge() {
        EdgeData duplicate_edge(edge0.start_point, edge0.direction, edge0.length);
        duplicate_edge.id = edge_space_dynamics.set_edge3d(duplicate_edge.start_point, duplicate_edge.direction, duplicate_edge.length);

        frame0.edge_nodes.push_back(EdgeNode(frame0.edge_nodes[0].direction_frame_to_edge, frame0.edge_nodes[0].edge_direction, duplicate_edge.id));
        frame1.edge_nodes.push_back(EdgeNode(frame1.edge_nodes[0].direction_frame_to_edge, frame1.edge_nodes[0].edge_direction, duplicate_edge.id));
        frame2.edge_nodes.push_back(EdgeNode(frame2.edge_nodes[0].direction_frame_to_edge, frame2.edge_nodes[0].edge_direction, duplicate_edge.id));
        frame3.edge_nodes.push_back(EdgeNode(frame3.edge_nodes[0].direction_frame_to_edge, frame3.edge_nodes[0].edge_direction, duplicate_edge.id));
        return duplicate_edge.id;
    }

    void addNoise(FrameData& frame, Eigen::Vector3f translation_noise, Eigen::Vector3f rotation_noise) {
        frame.pose.translate(translation_noise);
        frame.pose.rotate(rotation_noise);
    }

    void addNoise(EdgeData& edge, Eigen::Vector3f start_point_noise, Eigen::Vector3f direction_noise, float length_noise) {
        edge.start_point += start_point_noise;
        edge.direction += direction_noise;
        edge.length += length_noise;
    }

    void checkFrameData(const FrameData& frame, float translation_error_threshold = 1e-2, float rotation_error_threshold = 1e-2) {
        EXPECT_NEAR(frame.pose.position.x(), frame.correct_pose.position.x(), translation_error_threshold);
        EXPECT_NEAR(frame.pose.position.y(), frame.correct_pose.position.y(), translation_error_threshold);
        EXPECT_NEAR(frame.pose.position.z(), frame.correct_pose.position.z(), translation_error_threshold);

        EXPECT_NEAR(frame.pose.orientation.x(), frame.correct_pose.orientation.x(), rotation_error_threshold);
        EXPECT_NEAR(frame.pose.orientation.y(), frame.correct_pose.orientation.y(), rotation_error_threshold);
        EXPECT_NEAR(frame.pose.orientation.z(), frame.correct_pose.orientation.z(), rotation_error_threshold);
        EXPECT_NEAR(frame.pose.orientation.w(), frame.correct_pose.orientation.w(), rotation_error_threshold);
    }

    void checkEdgeData(const EdgeData& edge, float translation_error_threshold = 1e-1, float rotation_error_threshold = 1e-2) {
        EXPECT_NEAR(edge_space_dynamics.get_edge3d(edge.id).direction().x(), edge.correct_direction.x(), translation_error_threshold);
        EXPECT_NEAR(edge_space_dynamics.get_edge3d(edge.id).direction().y(), edge.correct_direction.y(), translation_error_threshold);
        EXPECT_NEAR(edge_space_dynamics.get_edge3d(edge.id).direction().z(), edge.correct_direction.z(), translation_error_threshold);
    }

    void checkEdgeNotIncluded(const FrameData& frame, int edge_id) {
        for (const auto& edge_node : frame.edge_nodes) {
            EXPECT_NE(edge_node.edge_id, edge_id);
        }
    }

    void removeInvalidEdge(FrameData& frame) {
        std::vector<EdgeNode> valid_edge_nodes;
        for (const auto& edge_node : frame.edge_nodes) {
            if (edge_node.is_valid) valid_edge_nodes.push_back(edge_node);
        }
        frame.edge_nodes = valid_edge_nodes;
    }

    void addNoiseForAllFrames() {
        addNoise(frame0, Eigen::Vector3f(0.2f, 0.1f, 0.3f), Eigen::Vector3f(0.1f, 0.2f, 0.2f));
        addNoise(frame1, Eigen::Vector3f(-1.2f, 0.1f, -0.3f), Eigen::Vector3f(1.1f, 0.3f, 0.2f));
        addNoise(frame2, Eigen::Vector3f(0.2f, 1.1f, 0.3f), Eigen::Vector3f(0.0f, 0.2f, 1.2f));
        addNoise(frame3, Eigen::Vector3f(0.2f, 0.0f, 0.5f), Eigen::Vector3f(-1.0f, -0.2f, 0.1f));
    }

    void addNoiseForAllEdges() {
        addNoise(edge0, Eigen::Vector3f(0.1f, 0.1f, 0.1f), Eigen::Vector3f(0.4f, -0.1f, 1.1f), 0.1f);
        addNoise(edge1, Eigen::Vector3f(-0.1f, 0.1f, -0.1f), Eigen::Vector3f(-1.1f, -0.1f, 0.4f), 0.1f);
        addNoise(edge2, Eigen::Vector3f(0.3f, 1.1f, 0.1f), Eigen::Vector3f(1.1f, 0.3f, -0.3f), 0.1f);
        addNoise(edge3, Eigen::Vector3f(0.1f, -1.1f, 0.1f), Eigen::Vector3f(-0.1f, -0.4f, 0.7f), 0.1f);
    }

    void optimizeAllFrames() {
        EXPECT_TRUE(edge_space_dynamics.optimize(frame0.pose, frame0.edge_nodes, Pose3D(), false));
        EXPECT_TRUE(edge_space_dynamics.optimize(frame1.pose, frame1.edge_nodes, frame1.correct_pose, true));
        EXPECT_TRUE(edge_space_dynamics.optimize(frame2.pose, frame2.edge_nodes, frame2.correct_pose, true));
        EXPECT_TRUE(edge_space_dynamics.optimize(frame3.pose, frame3.edge_nodes, frame3.correct_pose, true));
    }

    void removeInvalidEdgeFromAllFrames() {
        removeInvalidEdge(frame0);
        removeInvalidEdge(frame1);
        removeInvalidEdge(frame2);
        removeInvalidEdge(frame3);
    }

    void checkAllData() {
        checkFrameData(frame0);
        checkFrameData(frame1);
        checkFrameData(frame2);
        checkFrameData(frame3);

        checkEdgeData(edge0);
        checkEdgeData(edge1);
        checkEdgeData(edge2);
        checkEdgeData(edge3);
    }
};

TEST_F(OptimizeTest, useExternalPoseData) {
    int max_iterations = 2000;
    addNoiseForAllFrames();
    addNoiseForAllEdges();
    setEdges();
    for (int i = 0; i < max_iterations; i++) {
        optimizeAllFrames();
    }
    checkAllData();
}


TEST_F(OptimizeTest, removeInvalidEdge) {
    int max_iterations = 4000;
    addNoiseForAllFrames();
    addNoiseForAllEdges();
    setEdges();
    // add invalid edge
    // TODO: add more invalid edges
    int invalid_edge_id = addInvalidEdge();
    // optimize
    for (int i = 0; i < max_iterations; i++) {
        optimizeAllFrames();
        removeInvalidEdgeFromAllFrames();
    }

    // check if invalid edge is removed
    checkEdgeNotIncluded(frame0, invalid_edge_id);
    checkEdgeNotIncluded(frame1, invalid_edge_id);
    checkEdgeNotIncluded(frame2, invalid_edge_id);
    checkEdgeNotIncluded(frame3, invalid_edge_id);

    // check if valid edges are optimized
    checkAllData();
}

TEST_F(OptimizeTest, joinEdges) {
    int max_iterations = 4000;
    addNoiseForAllFrames();
    addNoiseForAllEdges();
    setEdges();
    int duplicate_edge_id = addDuplicateEdge();

    for (int i = 0; i < max_iterations; i++) {
        optimizeAllFrames();
    }

    // check if duplicate edge is joined
    checkEdgeNotIncluded(frame0, duplicate_edge_id);
    checkEdgeNotIncluded(frame1, duplicate_edge_id);
    checkEdgeNotIncluded(frame2, duplicate_edge_id);
    checkEdgeNotIncluded(frame3, duplicate_edge_id);

    EXPECT_EQ(frame0.edge_nodes[4].edge_id, edge0.id);
    EXPECT_EQ(frame1.edge_nodes[4].edge_id, edge0.id);
    EXPECT_EQ(frame2.edge_nodes[4].edge_id, edge0.id);
    EXPECT_EQ(frame3.edge_nodes[4].edge_id, edge0.id);

    checkAllData();
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
