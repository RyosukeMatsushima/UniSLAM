#include <gtest/gtest.h>
#include "edge_space_dynamics.hpp"

#define CONFIG_YAML_PATH PROJECT_SOURCE_DIR "/config_for_test.yaml"

class GetFramePoseTest : public ::testing::Test {
protected:
    EdgeSpaceDynamics edge_space_dynamics;
    std::vector<EdgeNode> edge_nodes;

    GetFramePoseTest() : edge_space_dynamics(CONFIG_YAML_PATH) {}

    void SetUp() override {
        edge_nodes = {
            createEdgeNode(1.0, 0.0, 1.0, 0.0, 1.0, edge_space_dynamics.set_edge3d(Eigen::Vector3f(1.0, 0.0, 0.0), Eigen::Vector3f(0.0, 1.0, 0.0), 1.0)),
            createEdgeNode(-1.0, 0.0, 1.0, 0.0, 1.0, edge_space_dynamics.set_edge3d(Eigen::Vector3f(-1.0, 0.0, 0.0), Eigen::Vector3f(0.0, 1.0, 0.0), 1.0)),
            createEdgeNode(0.0, 1.0, 1.0, 1.0, 0.0, edge_space_dynamics.set_edge3d(Eigen::Vector3f(0.0, 1.0, 0.0), Eigen::Vector3f(1.0, 0.0, 0.0), 1.0)),
            createEdgeNode(0.0, -1.0, 1.0, 1.0, 0.0, edge_space_dynamics.set_edge3d(Eigen::Vector3f(0.0, -1.0, 0.0), Eigen::Vector3f(1.0, 0.0, 0.0), 1.0))
        };
    }

    EdgeNode createEdgeNode(float df_x, float df_y, float df_z, float ed_x, float ed_y, int edge_id) {
        return EdgeNode(Eigen::Vector3f(df_x, df_y, df_z), Eigen::Vector2f(ed_x, ed_y), edge_id);
    }

    void addWrongEdgeNodeToBeginAndEnd() {
        edge_nodes.insert(edge_nodes.begin(), createEdgeNode(1.0, 1.0, 1.0, 0.0, 1.0, edge_space_dynamics.set_edge3d(Eigen::Vector3f(1.0, -1.0, 0.0), Eigen::Vector3f(0.0, 3.0, 0.0), 1.0)));
        edge_nodes.push_back(createEdgeNode(1.0, 0.0, 2.0, 1.0, 1.0, edge_space_dynamics.set_edge3d(Eigen::Vector3f(1.2, 0.1, 2.0), Eigen::Vector3f(0.1, -1.0, 0.0), 1.0)));
    }

    void checkFramePose(const Pose3D& frame_pose, const Eigen::Vector3f& expected_position, const Eigen::Quaternionf& expected_orientation, float position_error_threshold = 1e-4, float rotate_error_threshold = 1e-4) {
        EXPECT_NEAR(frame_pose.position.x(), expected_position.x(), position_error_threshold);
        EXPECT_NEAR(frame_pose.position.y(), expected_position.y(), position_error_threshold);
        EXPECT_NEAR(frame_pose.position.z(), expected_position.z(), position_error_threshold);

        EXPECT_NEAR(frame_pose.orientation.x(), expected_orientation.x(), rotate_error_threshold);
        EXPECT_NEAR(frame_pose.orientation.y(), expected_orientation.y(), rotate_error_threshold);
        EXPECT_NEAR(frame_pose.orientation.z(), expected_orientation.z(), rotate_error_threshold);
        EXPECT_NEAR(frame_pose.orientation.w(), expected_orientation.w(), rotate_error_threshold);
    }
};

TEST_F(GetFramePoseTest, calculateFramePose) {
    Pose3D frame_pose;
    frame_pose.translate(Eigen::Vector3f(0.0f, 0.0f, -1.2f));
    frame_pose.rotate(Eigen::Vector3f(0.1f, 0.0f, 0.0f));

    bool result = edge_space_dynamics.calculate_frame_pose(edge_nodes, frame_pose);

    Eigen::Vector3f expected_position(0.0, 0.0, -1.0);
    Eigen::Quaternionf expected_orientation = Eigen::Quaternionf::Identity();

    EXPECT_TRUE(result);
    checkFramePose(frame_pose, expected_position, expected_orientation);
}

TEST_F(GetFramePoseTest, calculateFramePoseHorizontalMoveTest) {
    Pose3D frame_pose;
    frame_pose.translate(Eigen::Vector3f(0.1f, 1.0f, -1.2f));
    frame_pose.rotate(Eigen::Vector3f(1.0f, -2.0f, 1.1f));

    bool result = edge_space_dynamics.calculate_frame_pose(edge_nodes, frame_pose);

    Eigen::Vector3f expected_position(0.0, 0.0, -1.0);
    Eigen::Quaternionf expected_orientation = Eigen::Quaternionf::Identity();

    EXPECT_TRUE(result);
    checkFramePose(frame_pose, expected_position, expected_orientation);
}

TEST_F(GetFramePoseTest, getFramePoseWithNoise) {
    Pose3D frame_pose;
    frame_pose.translate(Eigen::Vector3f(0.1f, 1.0f, -1.2f));
    frame_pose.rotate(Eigen::Vector3f(1.0f, -2.0f, 1.1f));

    addWrongEdgeNodeToBeginAndEnd();

    const float valid_edge_nodes_ratio_threshold = 0.5;

    bool result = edge_space_dynamics.get_frame_pose(edge_nodes,
                                                     valid_edge_nodes_ratio_threshold,
                                                     frame_pose);

    Eigen::Vector3f expected_position(0.0, 0.0, -1.0);
    Eigen::Quaternionf expected_orientation = Eigen::Quaternionf::Identity();

    EXPECT_TRUE(result);
    checkFramePose(frame_pose, expected_position, expected_orientation);

    // edge_node should be valid except for the first and last edge_node
    for (int i = 0; i < edge_nodes.size(); i++) {
        if (i == 0 || i == edge_nodes.size() - 1) {
            EXPECT_FALSE(edge_nodes[i].is_valid);
        } else {
            EXPECT_TRUE(edge_nodes[i].is_valid);
        }
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
