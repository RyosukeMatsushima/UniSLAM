#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "discrete_angle_edge_intensity.hpp"

#define TEST_IMAGE_PATH "./result/discrete_angle_edge_intensity/"


TEST(DiscreteAngleEdgeIntensityTest, TestDiscreteAngleEdgeIntensity) {

    float angle_resolution = 0.9f;

    int window_size = int(2 * M_PI / angle_resolution);

    cv::Mat intensity_map = cv::Mat::zeros(window_size, window_size, CV_32F);

    cv::Mat gradient_angle_map = cv::Mat::zeros(window_size, window_size, CV_32F);

    for (int i = 0; i < window_size; i++) {
        float angle = i * angle_resolution;

        intensity_map.at<float>(i, i) = 1.0f;
        gradient_angle_map.at<float>(i, i) = angle;
    }

    DiscreteAngleEdgeIntensity discrete_angle_edge_intensity(intensity_map,
                                                             gradient_angle_map,
                                                             angle_resolution);

    // test the discretizeAngle function with angle range 0
    for (int i = 2; i < window_size - 2; i++) {
        cv::Mat intensity = discrete_angle_edge_intensity.getBlockIntensity(cv::Point(i, i), 3, i * angle_resolution, 0);

        cv::Mat expected_intensity = cv::Mat::zeros(3, 3, CV_32F);
        expected_intensity.at<float>(1, 1) = 1.0f;

        cv::Mat diff;
        cv::absdiff(intensity, expected_intensity, diff);

        cv::Mat mask = diff > 1e-6;

        EXPECT_EQ(cv::countNonZero(mask), 0);
    }

    // test the discretizeAngle function with angle range 1
    for (int i = 2; i < window_size - 2; i++) {
        cv::Mat intensity = discrete_angle_edge_intensity.getBlockIntensity(cv::Point(i, i), 3, i * angle_resolution, 1);

        cv::Mat expected_intensity = cv::Mat::zeros(3, 3, CV_32F);
        expected_intensity.at<float>(0, 0) = 1.0f;
        expected_intensity.at<float>(1, 1) = 1.0f;
        expected_intensity.at<float>(2, 2) = 1.0f;

        cv::Mat diff;
        cv::absdiff(intensity, expected_intensity, diff);

        cv::Mat mask = diff > 1e-6;

        EXPECT_EQ(cv::countNonZero(mask), 0);
    }

    // test the discretizeAngle function with angle range 2
    for (int i = 2; i < window_size - 2; i++) {
        cv::Mat intensity = discrete_angle_edge_intensity.getBlockIntensity(cv::Point(i, i), 5, i * angle_resolution, 2);

        cv::Mat expected_intensity = cv::Mat::zeros(5, 5, CV_32F);
        expected_intensity.at<float>(0, 0) = 1.0f;
        expected_intensity.at<float>(1, 1) = 1.0f;
        expected_intensity.at<float>(2, 2) = 1.0f;
        expected_intensity.at<float>(3, 3) = 1.0f;
        expected_intensity.at<float>(4, 4) = 1.0f;

        cv::Mat diff;
        cv::absdiff(intensity, expected_intensity, diff);

        cv::Mat mask = diff > 1e-6;

        EXPECT_EQ(cv::countNonZero(mask), 0);
    }
}

TEST(DiscretizeAngleTest, TestDiscretizeAngle) {
    float angle_resolution = 0.1f;
    cv::Mat intensity_map = cv::Mat::zeros(1, 1, CV_32F);
    cv::Mat gradient_angle_map = cv::Mat::zeros(1, 1, CV_32F);
    DiscreteAngleEdgeIntensity discrete_angle_edge_intensity(intensity_map,
                                                             gradient_angle_map,
                                                             angle_resolution);

    // test the discretizeAngle function with small range of angle
    for (int i = 0; i < 10; i++) {
        float angle = i * angle_resolution;
        int angle_index = discrete_angle_edge_intensity.discretizeAngle(angle);

        EXPECT_EQ(angle_index, i);
    }
}
