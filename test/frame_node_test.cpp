#include <gtest/gtest.h>

#include "frame_node.hpp"
#include "debug_view.hpp"

#define RESULT_IMAGE_PATH "./result/frame_node/"

TEST(FrameNodeTest, MatchEdge) {
    int image_size = 1000;
    int rectangle_size = 100;

    int window_size = 10;
    float angle_resolution = 0.2;

    cv::Mat test_img = cv::Mat::zeros(image_size, image_size, CV_8UC1);

    cv::rectangle(test_img, cv::Point(0, 0), cv::Point(rectangle_size, rectangle_size), cv::Scalar(255), -1);

    FrameNode frame_node(test_img,
                         window_size,
                         angle_resolution);

    EdgePoint expected_edge_point(cv::Point2f(rectangle_size, rectangle_size / 2), cv::Vec2f(-1, 0));

    EdgePoint matched_edge_point(cv::Point2f(0, 0), cv::Vec2f(0, 0));

    bool result = frame_node.matchEdge(expected_edge_point, matched_edge_point);

    ASSERT_TRUE(result);
    ASSERT_NEAR(expected_edge_point.point.x, matched_edge_point.point.x, 2);
    ASSERT_NEAR(expected_edge_point.angle, matched_edge_point.angle, 1e-5);
}

TEST(FrameNodeTest, NewEdgePointsNotIncludingFixedEdgePoints) {
    int image_size = 1000;
    int circle_radius = 300;

    int window_size = 50;
    float angle_resolution = 0.2;

    cv::Mat test_img = cv::Mat::zeros(image_size, image_size, CV_8UC1);

    // draw a circle
    cv::circle(test_img, cv::Point(image_size / 2, image_size / 2), circle_radius, cv::Scalar(255), -1);

    FrameNode frame_node(test_img,
                         window_size,
                         angle_resolution);

    std::vector<EdgePoint> new_edge_points = frame_node.findNewEdgePoints();

    // number of new edge points should be more than 0
    ASSERT_TRUE(new_edge_points.size() > 0);

    std::vector<EdgePoint> removed_edge_points;
    std::vector<EdgePoint> fixed_edge_points;

    // remove edge points every 3rd point
    for (int i = 0; i < new_edge_points.size(); i++) {
        if (new_edge_points[i].point.x > image_size / 2) {
            removed_edge_points.push_back(new_edge_points[i]);
        } else {
            fixed_edge_points.push_back(new_edge_points[i]);
        }
    }

    // add new edge points as fixed edge points
    for (const EdgePoint& edge_point : fixed_edge_points) {
        frame_node.addFixedEdgePoint(edge_point);
    }

    std::vector<EdgePoint> new_edge_points_after_adding_fixed_edge_points = frame_node.findNewEdgePoints();

    // Save the image
    DebugView debug_view(test_img);
    debug_view.drawEdgePoints(new_edge_points_after_adding_fixed_edge_points);
    cv::imwrite(RESULT_IMAGE_PATH "new_edge_points_not_including_fixed_edge_points.png", debug_view.getDebugImage());

    DebugView debug_view2(test_img);
    debug_view2.drawEdgePoints(removed_edge_points);
    cv::imwrite(RESULT_IMAGE_PATH "removed_edge_points.png", debug_view2.getDebugImage());

    DebugView debug_view3(test_img);
    debug_view3.drawEdgePoints(fixed_edge_points);
    cv::imwrite(RESULT_IMAGE_PATH "fixed_edge_points.png", debug_view3.getDebugImage());

    DebugView debug_view4(test_img);
    debug_view4.drawEdgePoints(new_edge_points);
    cv::imwrite(RESULT_IMAGE_PATH "new_edge_points.png", debug_view4.getDebugImage());

    // check size
    ASSERT_EQ(new_edge_points.size(), new_edge_points_after_adding_fixed_edge_points.size() + fixed_edge_points.size());

    for (const EdgePoint& new_edge_point : new_edge_points_after_adding_fixed_edge_points) {
        for (const EdgePoint& fixed_edge_point : frame_node.getFixedEdgePoints()) {
            ASSERT_FALSE(new_edge_point.point == fixed_edge_point.point);
        }
    }
}

