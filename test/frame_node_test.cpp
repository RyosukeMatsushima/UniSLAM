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

TEST(FrameNodeTest, MatchEdgeWithSameImg) {
    int image_size = 1000;
    int circle_radius = 300;

    int window_size = 50;
    float angle_resolution = 0.2;

    cv::Mat test_img = cv::Mat::zeros(image_size, image_size, CV_8UC1);
    cv::circle(test_img, cv::Point(image_size / 2, image_size / 2), circle_radius, cv::Scalar(255), -1);

    FrameNode frame_node1(test_img,
                          window_size,
                          angle_resolution);

    std::vector<EdgePoint> new_edge_points1 = frame_node1.findNewEdgePoints();

    // number of new edge points should be more than 0
    ASSERT_TRUE(new_edge_points1.size() > 0);

    FrameNode frame_node2(test_img,
                          window_size,
                          angle_resolution);

    // all edge points should be matched
    for (EdgePoint& edge_point : new_edge_points1) {
        EdgePoint matched_edge_point(cv::Point2f(0, 0), cv::Vec2f(0, 0));
        bool result = frame_node2.matchEdge(edge_point, matched_edge_point);

        // ASSERT_TRUE(result); // TODO: fix this
    }
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

cv::Mat getTestImgWithLine(const int image_size,
                           const int line_length,
                           const int line_width,
                           const cv::Point shift_point,
                           const float line_angle) {
    cv::Mat test_img = cv::Mat::zeros(image_size, image_size, CV_8UC1);

    cv::Point center = cv::Point(test_img.cols / 2, test_img.rows / 2);

    cv::Point start_point = center + shift_point + cv::Point(cos(line_angle) * line_length, sin(line_angle) * line_length);
    cv::Point end_point = center + shift_point - cv::Point(cos(line_angle) * line_length, sin(line_angle) * line_length);

    cv::line(test_img, start_point, end_point, cv::Scalar(255), line_width);

    return test_img;
}

TEST(FrameNodeTest, ChecKeyFrameItself) {
    int image_size = 1000;
    int line_length = 300;
    int line_width = 100;

    int window_size = 50;
    float angle_resolution = 0.2;

    // draw a line
    cv::Mat base_test_img = getTestImgWithLine(image_size, line_length, line_width, cv::Point(0, 0), 0);

    FrameNode frame_node1(base_test_img,
                          window_size,
                          angle_resolution);

    std::vector<EdgePoint> new_edge_points1 = frame_node1.findNewEdgePoints();
    for (const EdgePoint& edge_point : new_edge_points1) {
        frame_node1.addFixedEdgePoint(edge_point);
    }

    // number of new edge points should be more than 0
    ASSERT_GE(frame_node1.getFixedEdgePoints().size(), 1);

    // test with no shifted image
    FrameNode frame_node2(base_test_img,
                          window_size,
                          angle_resolution);
    // match edge points to base frame
    bool is_key_frame = false;
    bool result = frame_node2.matchWith(frame_node1, is_key_frame);

    // create debug view
    DebugView debug_view(base_test_img);
    //debug_view.drawEdgePoints(new_edge_points1, cv::Scalar(255, 0, 0));
    debug_view.drawEdgePoints(frame_node2.getFixedEdgePoints(), cv::Scalar(0, 255, 0));
    cv::imwrite(RESULT_IMAGE_PATH "check_is_key_frame_itself.png", debug_view.getDebugImage());

    // check result is not key frame
    EXPECT_GE(frame_node2.getFixedEdgePoints().size(), 1);
    ASSERT_FALSE(is_key_frame);
    ASSERT_TRUE(result);


    // test with shifted image
    cv::Mat holizontal_shift_test_img = getTestImgWithLine(image_size, line_length, line_width, cv::Point(0, window_size * 0.4), 0);
    FrameNode frame_node3(holizontal_shift_test_img,
                          window_size,
                          angle_resolution);
    // match edge points to base frame
    result = frame_node3.matchWith(frame_node1, is_key_frame);

    // create debug view
    DebugView debug_view2(holizontal_shift_test_img);
    debug_view2.drawEdgePoints(new_edge_points1, cv::Scalar(255, 0, 0));
    debug_view2.drawEdgePoints(frame_node3.getFixedEdgePoints(), cv::Scalar(0, 255, 0));
    cv::imwrite(RESULT_IMAGE_PATH "check_is_key_frame_itself_shifted.png", debug_view2.getDebugImage());
    // check result is key frame
    ASSERT_TRUE(is_key_frame);
    ASSERT_TRUE(result);

    // test with rotated image
    // TODO: add test for rotated image

    // test with scaled image
    // TODO: add test for scaled image

    // test with black image
    cv::Mat black_test_img = cv::Mat::zeros(image_size, image_size, CV_8UC1);
    FrameNode frame_node4(black_test_img,
                          window_size,
                          angle_resolution);
    // match edge points to base frame
    result = frame_node4.matchWith(frame_node1, is_key_frame);

    // create debug view
    DebugView debug_view3(black_test_img);
    debug_view3.drawEdgePoints(new_edge_points1, cv::Scalar(255, 0, 0));
    debug_view3.drawEdgePoints(frame_node4.getFixedEdgePoints(), cv::Scalar(0, 255, 0));
    cv::imwrite(RESULT_IMAGE_PATH "check_is_key_frame_itself_black.png", debug_view3.getDebugImage());

    // check result is not key frame
    ASSERT_FALSE(is_key_frame);
    ASSERT_FALSE(result);
}

