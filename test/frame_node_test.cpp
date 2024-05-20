#include <gtest/gtest.h>

#include "frame_node.hpp"

TEST(FrameNodeTest, MatchEdge) {
    int image_size = 1000;
    int rectangle_size = 100;

    cv::Mat test_img = cv::Mat::zeros(image_size, image_size, CV_8UC1);

    cv::rectangle(test_img, cv::Point(0, 0), cv::Point(rectangle_size, rectangle_size), cv::Scalar(255), -1);

    FrameNode frame_node(test_img);

    EdgePoint expected_edge_point(cv::Point2f(rectangle_size, rectangle_size / 2), cv::Vec2f(-1, 0));

    EdgePoint matched_edge_point(cv::Point2f(0, 0), cv::Vec2f(0, 0));

    bool result = frame_node.matchEdge(expected_edge_point, matched_edge_point);

    ASSERT_TRUE(result);
    ASSERT_NEAR(expected_edge_point.point.x, matched_edge_point.point.x, 2);
    ASSERT_NEAR(expected_edge_point.angle, matched_edge_point.angle, 1e-5);
}

