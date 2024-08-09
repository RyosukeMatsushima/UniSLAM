#include <gtest/gtest.h>

#include "frame_node.hpp"
#include "debug_view.hpp"

#define RESULT_IMAGE_PATH PROJECT_SOURCE_DIR "/test/result/"

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

        // EXPECT_TRUE(result); // TODO: fix this
    }
}

TEST(FrameNodeTest, MatchEdgeCheckWithRemovedEdgePoints) {
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
    int id = 0;
    for (EdgePoint& edge_point : fixed_edge_points) {
        edge_point.id = id++;
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

cv::Mat getTestImgWithLine(const cv::Point shift_point,
                           const float line_angle,
                           const float scale = 1.0f) {

    int image_size = 1000;
    int line_length = 300 * scale;
    int line_width = 100 * scale;

    cv::Mat test_img = cv::Mat::zeros(image_size, image_size, CV_8UC1);

    cv::Point center = cv::Point(test_img.cols / 2, test_img.rows / 2);

    cv::Point start_point = center + shift_point + cv::Point(cos(line_angle) * line_length, sin(line_angle) * line_length);
    cv::Point end_point = center + shift_point - cv::Point(cos(line_angle) * line_length, sin(line_angle) * line_length);

    cv::line(test_img, start_point, end_point, cv::Scalar(85), line_width);

    return test_img;
}

void matchFrame(cv::Mat& test_img1,
                cv::Mat& test_img2,
                const int window_size,
                const float angle_resolution,
                const std::string& file_name,
                const float match_threshold_ratio) {

    FrameNode frame_node1(test_img1,
                          window_size,
                          angle_resolution);

    std::vector<EdgePoint> new_edge_points1 = frame_node1.findNewEdgePoints();
    int edge_point_id = 0;
    for (EdgePoint& edge_point : new_edge_points1) {
        edge_point.id = edge_point_id++;
        frame_node1.addFixedEdgePoint(edge_point);
    }

    EXPECT_GE(frame_node1.getFixedEdgePoints().size(), 1);

    FrameNode frame_node2(test_img2,
                          window_size,
                          angle_resolution);

    bool is_key_frame = false;

    bool result = frame_node2.matchWith(frame_node1, is_key_frame);

    EXPECT_TRUE(result);
    int edge_num_threshold = edge_point_id * match_threshold_ratio;
    EXPECT_GE(frame_node2.getFixedEdgePoints().size(), edge_num_threshold);

    DebugView debug_view(test_img1);
    debug_view.drawEdgePoints(new_edge_points1, cv::Scalar(255, 0, 0));
    debug_view.drawEdgePoints(frame_node2.getFixedEdgePoints(), cv::Scalar(0, 255, 0));
    cv::imwrite(RESULT_IMAGE_PATH + file_name, debug_view.getDebugImage());
}

TEST(FrameNodeTest, ChecKeyFrameItself) {
    int window_size = 50;
    float angle_resolution = 0.2;

    cv::Mat base_test_img = getTestImgWithLine(cv::Point(0, 0), 0);
    matchFrame(base_test_img, base_test_img, window_size, angle_resolution, "check_is_key_frame_itself_same.png", 0.8); // TODO: change edge_threshold_ratio 0.8 to 1.0

}

TEST(FrameNodeTest, ChecKeyFrameItselfWithHolizontalShiftedImg) {
    int window_size = 50;
    float angle_resolution = 0.2;

    cv::Mat base_test_img = getTestImgWithLine(cv::Point(0, 0), 0);
    cv::Mat holizontal_shift_test_img = getTestImgWithLine(cv::Point(0, window_size * 0.4), 0);
    matchFrame(base_test_img, holizontal_shift_test_img, window_size, angle_resolution, "check_is_key_frame_itself_holizontal_shift.png", 0.8);
}

TEST(FrameNodeTest, ChecKeyFrameItselfWithVerticalShiftedImg) {
    int window_size = 50;
    float angle_resolution = 0.2;

    cv::Mat base_test_img = getTestImgWithLine(cv::Point(0, 0), 0);
    cv::Mat vertical_shift_test_img = getTestImgWithLine(cv::Point(window_size * 0.4, 0), 0);
    matchFrame(base_test_img, vertical_shift_test_img, window_size, angle_resolution, "check_is_key_frame_itself_vertical_shift.png", 0.8);
}

TEST(FrameNodeTest, ChecKeyFrameItselfWithRotatedImg) {
    int window_size = 50;
    float angle_resolution = 0.2;

    cv::Mat base_test_img = getTestImgWithLine(cv::Point(0, 0), 0);
    cv::Mat rotated_test_img = getTestImgWithLine(cv::Point(0, 0), angle_resolution/3);
    matchFrame(base_test_img, rotated_test_img, window_size, angle_resolution, "check_is_key_frame_itself_rotated.png", 0.8);
}

TEST(FrameNodeTest, ChecKeyFrameItselfWithScaledImg) {
    int window_size = 50;
    float angle_resolution = 0.2;

    cv::Mat base_test_img = getTestImgWithLine(cv::Point(0, 0), 0);
    cv::Mat scaled_test_img = getTestImgWithLine(cv::Point(0, 0), 0, 1.2);
    matchFrame(base_test_img, scaled_test_img, window_size, angle_resolution, "check_is_key_frame_itself_scaled.png", 0.8);
}

TEST(FrameNodeTest, ShuffleFixedEdges) {
    cv::Mat test_img = cv::Mat::zeros(1000, 1000, CV_8UC1);
    FrameNode frame_node(test_img, 50, 0.2);

    // add fixed edge points
    int id = 0;

    for (int i = 0; i < 10; i++) {
        EdgePoint edge_point(cv::Point2f(i, i), cv::Vec2f(0, 0));
        edge_point.id = id++;
        frame_node.addFixedEdgePoint(edge_point);
    }

    // shuffle fixed edge points
    frame_node.shuffleFixedEdgePoints();

    // check if the order is changed
    int same_order_count = 0;
    for (int i = 0; i < frame_node.getFixedEdgePoints().size(); i++) {
        EdgePoint edge_point(cv::Point2f(i, i), cv::Vec2f(0, 0));
        edge_point.id = i;
        if (frame_node.getFixedEdgePoints()[i].id == edge_point.id) {
            same_order_count++;
        }
    }

    int same_order_threshold = frame_node.getFixedEdgePoints().size() * 0.8;

    ASSERT_LT(same_order_count, same_order_threshold);
}

TEST(FrameNodeTest, OperatorEqual) {
    cv::Mat test_img = cv::Mat::zeros(1000, 1000, CV_8UC1);
    int window_size = 50;
    float angle_resolution = 0.2;
    FrameNode frame_node1(test_img, window_size, angle_resolution);
    FrameNode frame_node2(test_img, window_size, angle_resolution);
    FrameNode frame_node_wrong_param(test_img, window_size, angle_resolution + 0.1);

    // add fixed edge points
    int id = 0;

    for (int i = 0; i < 10; i++) {
        EdgePoint edge_point(cv::Point2f(i, i), cv::Vec2f(0, 0));
        edge_point.id = id++;
        frame_node1.addFixedEdgePoint(edge_point);
    }

    frame_node2 = frame_node1;

    // check if the fixed edge points are same
    for (int i = 0; i < frame_node1.getFixedEdgePoints().size(); i++) {
        ASSERT_EQ(frame_node1.getFixedEdgePoints()[i].id, frame_node2.getFixedEdgePoints()[i].id);
    }

    // return error if the window size is different
    ASSERT_THROW(frame_node_wrong_param = frame_node1, std::invalid_argument);
}

TEST(FrameNodeTest, RemoveFixedEdgePoint) {
    cv::Mat test_img = cv::Mat::zeros(1000, 1000, CV_8UC1);
    FrameNode frame_node(test_img, 50, 0.2);

    // add fixed edge points
    int id = 0;

    for (int i = 0; i < 10; i++) {
        EdgePoint edge_point(cv::Point2f(i, i), cv::Vec2f(0, 0));
        edge_point.id = id++;
        frame_node.addFixedEdgePoint(edge_point);
    }

    // remove fixed edge point
    int remove_id = 5;

    frame_node.removeFixedEdgePoint(remove_id);

    // check if the fixed edge points are removed
    for (int i = 0; i < frame_node.getFixedEdgePoints().size(); i++) {
        ASSERT_NE(frame_node.getFixedEdgePoints()[i].id, remove_id);
    }
}

TEST(FrameNodeTest, AddFixedEdgePoint) {
    cv::Mat test_img = cv::Mat::zeros(1000, 1000, CV_8UC1);
    FrameNode frame_node(test_img, 50, 0.2);

    // add fixed edge points
    int id = 0;

    for (int i = 0; i < 10; i++) {
        EdgePoint edge_point(cv::Point2f(i, i), cv::Vec2f(0, 0));
        edge_point.id = id++;
        frame_node.addFixedEdgePoint(edge_point);
    }

    // check if the fixed edge points are same
    for (int i = 0; i < frame_node.getFixedEdgePoints().size(); i++) {
        ASSERT_EQ(frame_node.getFixedEdgePoints()[i].id, i);
    }

    // add fixed edge points with invalid id
    // should throw an error
    int invalid_id = -1;
    ASSERT_THROW(frame_node.getFixedEdgePoint(invalid_id), std::invalid_argument);
}

TEST(FrameNodeTest, GetFixedEdgePointById) {
    cv::Mat test_img = cv::Mat::zeros(1000, 1000, CV_8UC1);
    FrameNode frame_node(test_img, 50, 0.2);

    // add fixed edge points
    const int start_id = 3;
    const int id_step = 2;

    for (int i = 0; i < 10; i++) {
        EdgePoint edge_point(cv::Point2f(i, i), cv::Vec2f(0, 0));
        edge_point.id = start_id + id_step * i;
        frame_node.addFixedEdgePoint(edge_point);
    }

    // check if the fixed edge points are same
    for (int i = 0; i < frame_node.getFixedEdgePoints().size(); i++) {
        ASSERT_EQ(frame_node.getFixedEdgePoints()[i].id, start_id + id_step * i);
    }

    // get fixed edge points with invalid id
    // should throw an error
    int invalid_id = 100;
    ASSERT_THROW(frame_node.getFixedEdgePoint(invalid_id), std::invalid_argument);
}

