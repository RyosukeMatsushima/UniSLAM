#include <gtest/gtest.h>
#include "edge_space_dynamics.hpp"

#define CONFIG_YAML_PATH PROJECT_SOURCE_DIR "/config_for_test.yaml"

TEST(EdgeControlTest, setGetRemoveEdge) {
    EdgeSpaceDynamics edge_space_dynamics(CONFIG_YAML_PATH);

    Eigen::Vector3f start_point(1.0, 0.0, 1.0);
    Eigen::Vector3f direction(0.0, 1.0, 0.0);

    int edge_num = 5;

    for (int i = 0; i < edge_num; i++) {
        float length = float(i);
        int edge_id = edge_space_dynamics.set_edge3d(start_point, direction, length);
        EXPECT_EQ(edge_id, i);
        EXPECT_EQ(edge_space_dynamics.get_edge3d(edge_id).length(), length);
    }

    int remove_edge_id = 2;
    edge_space_dynamics.remove_edge3d(remove_edge_id);

    EXPECT_THROW(edge_space_dynamics.get_edge3d(remove_edge_id), std::invalid_argument);

    for (int i = 0; i < edge_num - 1; i++) {
        if (i == remove_edge_id) continue;
        EXPECT_EQ(edge_space_dynamics.get_edge3d(i).length(), float(i));
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
