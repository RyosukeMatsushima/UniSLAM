#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "edge_point_check.hpp"

#define RESULT_IMAGE_PATH "./result/edge_point_check/"

cv::Mat getTestImage(int line_width,
                     int line_length,
                     cv::Vec2f line_direction,
                     cv::Size image_size,
                     int background_intensity = 0) {
    cv::Mat image = cv::Mat::zeros(image_size, CV_8U);
    image = cv::Scalar(background_intensity);

    cv::Point2f center = cv::Point2f(image_size.width / 2, image_size.height / 2);
    cv::Point start(center.x + line_direction[0] * line_length / 2,
                    center.y + line_direction[1] * line_length / 2);
    cv::Point end(center.x - line_direction[0] * line_length / 2,
                  center.y - line_direction[1] * line_length / 2);

    cv::line(image, start, end, cv::Scalar(255), line_width);

    return image;
}

TEST(EdgePointCheckTest, TestEdgePointCheck) {
    int image_width = 100;
    int image_height = 100;

    cv::Size image_size(image_width, image_height);
    cv::Vec2f edge_direction(1, 1);
    cv::Mat image = getTestImage(1,
                                 std::min(image_width, image_height),
                                 edge_direction,
                                 image_size);

    EdgePointCheck edge_point_check(image, edge_direction);

    bool is_valid = edge_point_check.is_valid();

    EXPECT_TRUE(is_valid);
}

TEST(EdgePointCheckTest, TestOverThresholdCount) {
    cv::Size image_size(10, 10);
    cv::Mat image = cv::Mat::zeros(image_size, CV_8U);

    int threshold = 50;
    int over_threshold_count = 6;

    for (int i = 0; i < over_threshold_count; i++) {
        image.at<int>(i, i) = 255;
    }

    EdgePointCheck edge_point_check(image, cv::Vec2f(1, 1));

    int result = edge_point_check.over_threshold_count(image, threshold);

    EXPECT_EQ(over_threshold_count, result);
}

TEST(EdgePointCheckTest, TestEdgePointCheckInvalid) {
    int image_width = 100;
    int image_height = 100;

    cv::Size image_size(image_width, image_height);
    cv::Vec2f edge_direction(1, 1);
    cv::Mat image = getTestImage(1,
                                 std::min(image_width, image_height),
                                 edge_direction,
                                 image_size);

    cv::Vec2f edge_direction_invalid(1, -1);
    EdgePointCheck edge_point_check(image, edge_direction_invalid);

    bool is_valid = edge_point_check.is_valid();

    EXPECT_FALSE(is_valid);
}

TEST(EdgePointCheckTest, TestRemoveUnusedArea) {
    cv::Size image_size(100, 100);
    cv::Mat image = getTestImage(1, 100, cv::Vec2f(1, 1), image_size, 100);

    EdgePointCheck edge_point_check(image, cv::Vec2f(1, 1));

    cv::Mat result = edge_point_check.remove_unused_area(image);

    cv::imwrite(RESULT_IMAGE_PATH "remove_unused_area.jpg", result);
}

TEST(EdgePointCheckTest, TestGetDesiredEdgeArea) {
    cv::Size image_size(100, 100);
    cv::Vec2f edge_direction(1, 1);
    cv::Mat image = getTestImage(1, 100, edge_direction, image_size, 100);

    EdgePointCheck edge_point_check(image, edge_direction);

    cv::Mat result = edge_point_check.get_desired_edge_area(image);

    cv::imwrite(RESULT_IMAGE_PATH "get_desired_edge_area.jpg", result);
    cv::imwrite(RESULT_IMAGE_PATH "get_desired_edge_area_original.jpg", image);
}


