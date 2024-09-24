#include "edge_distribution.hpp"
#include <gtest/gtest.h>

TEST(EdgeDistributionTest, AddEdgePoint) {
    cv::Size imgSize(640, 480);
    cv::Size windowSize(32, 32);
    EdgeDistribution edgeDistribution(imgSize, windowSize);

    cv::Point edgePoint(100, 200);
    edgeDistribution.add_edge_point(edgePoint);

    // Assert that the edge number at the window containing the edge point is incremented by 1
    cv::Point window(edgePoint.x / windowSize.width, edgePoint.y / windowSize.height);
    int expectedEdgeNum = 1;
    int actualEdgeNum = edgeDistribution.edge_num(edgePoint);
    EXPECT_EQ(actualEdgeNum, expectedEdgeNum);
}

TEST(EdgeDistributionTest, RemoveEdgePoint) {
    cv::Size imgSize(640, 480);
    cv::Size windowSize(32, 32);
    EdgeDistribution edgeDistribution(imgSize, windowSize);

    cv::Point edgePoint(100, 200);
    edgeDistribution.add_edge_point(edgePoint);
    edgeDistribution.remove_edge_point(edgePoint);

    // Assert that the edge number at the window containing the edge point is decremented by 1
    cv::Point window(edgePoint.x / windowSize.width, edgePoint.y / windowSize.height);
    int expectedEdgeNum = 0;
    int actualEdgeNum = edgeDistribution.edge_num(edgePoint);
    EXPECT_EQ(actualEdgeNum, expectedEdgeNum);
}

TEST(EdgeDistributionTest, GetEmptyWindows) {
    cv::Size imgSize(640, 480);
    cv::Size windowSize(32, 32);
    EdgeDistribution edgeDistribution(imgSize, windowSize);

    cv::Point edgePoint(100, 200);
    edgeDistribution.add_edge_point(edgePoint);

    // Assert that the window containing the edge point is not empty
    std::vector<cv::Point> emptyWindows = edgeDistribution.get_empty_windows();
    EXPECT_FALSE(std::find(emptyWindows.begin(), emptyWindows.end(), edgePoint) != emptyWindows.end());
}

TEST(EdgeDistributionTest, EdgeNumOutOfBounds) {
    cv::Size imgSize(640, 480);
    cv::Size windowSize(32, 32);
    EdgeDistribution edgeDistribution(imgSize, windowSize);

    cv::Point outOfBoundsPoint(1000, 1000);

    // Assert that an invalid_argument exception is thrown when accessing edge number for an out-of-bounds point
    EXPECT_THROW(edgeDistribution.edge_num(outOfBoundsPoint), std::invalid_argument);
}

// Add more tests as needed

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
