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
};

TEST_F(OptimizeTest, optimizeWithoutNoise) {
    int max_iterations = 2000;

    addNoise(frame0, Eigen::Vector3f(0.2f, 0.1f, 0.3f), Eigen::Vector3f(0.1f, 0.2f, 0.2f));

    addNoise(edge0, Eigen::Vector3f(0.1f, 0.1f, 0.1f), Eigen::Vector3f(0.4f, -0.1f, 1.1f), 0.1f);
    addNoise(edge1, Eigen::Vector3f(-0.1f, 0.1f, -0.1f), Eigen::Vector3f(-1.1f, -0.1f, 0.4f), 0.1f);
    addNoise(edge2, Eigen::Vector3f(0.3f, 1.1f, 0.1f), Eigen::Vector3f(1.1f, 0.3f, -0.3f), 0.1f);
    addNoise(edge3, Eigen::Vector3f(0.1f, -1.1f, 0.1f), Eigen::Vector3f(-0.1f, -0.4f, 0.7f), 0.1f);

    setEdges();

    for (int i = 0; i < max_iterations; i++) {
        EXPECT_TRUE(edge_space_dynamics.optimize(frame0.pose, frame0.edge_nodes, true));
        EXPECT_TRUE(edge_space_dynamics.optimize(frame1.pose, frame1.edge_nodes, false));
        EXPECT_TRUE(edge_space_dynamics.optimize(frame2.pose, frame2.edge_nodes, false));
        EXPECT_TRUE(edge_space_dynamics.optimize(frame3.pose, frame3.edge_nodes, false));
    }

    checkFrameData(frame0);
    checkFrameData(frame1);
    checkFrameData(frame2);

    checkEdgeData(edge0);
    checkEdgeData(edge1);
    checkEdgeData(edge2);
    checkEdgeData(edge3);
}

