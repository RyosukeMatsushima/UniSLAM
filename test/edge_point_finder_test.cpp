#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>

#include "edge_point_finder.hpp"
#include "edge.h"
#include "frame.h"
#include "debug_view.hpp"

#define RESULT_IMAGE_PATH "./result/edge_point_finder/"

TEST(WithSimpleLine, TestEdgePointFinder) {
    int image_width = 200;
    int edge_find_window_size = 30;

    for (float gradient_angle=0.0f; gradient_angle < 2 * M_PI; gradient_angle += M_PI / 6) {

        cv::Mat input_image = cv::Mat::zeros(image_width, image_width, CV_32F);

        // Draw a line through the center of the image with a angle
        float line_angle = gradient_angle + M_PI / 2;
        int x = image_width / 2 * cos(line_angle);
        int y = image_width / 2 * sin(line_angle);
        cv::Point start(image_width / 2 - x, image_width / 2 - y);
        cv::Point end(image_width / 2 + x, image_width / 2 + y);

        cv::line(input_image, start, end, cv::Scalar(255), 5);

        Frame frame(input_image);

        EdgePointFinder edge_point_finder;

        bool result = false;
        EdgePoint edge_point = edge_point_finder.find_key_edge_point(frame,
                                                                     cv::Point(image_width / 2, image_width / 2),
                                                                     gradient_angle,
                                                                     edge_find_window_size,
                                                                     result);
        // Create a DebugView
        DebugView debug_view(input_image);
        debug_view.drawEdgePoints({edge_point});

        // Save the image
        std::string img_name = std::to_string((int)(line_angle * 180 / M_PI)) + "_line.jpg";
        std::string img_path = RESULT_IMAGE_PATH;

        cv::imwrite(img_path + "edge_point_with_" + img_name, debug_view.getDebugImage());

        // result should be true
        ASSERT_TRUE(result);
    }
}


TEST(WithTestImg, TestEdgePointFinder) {

    int window_size = 1000;
    int edge_find_window_size = 20;

    // create a black image
    cv::Mat test_image = cv::Mat::zeros(window_size, window_size, CV_8UC3);
    // draw a white rectangle
    cv::rectangle(test_image, cv::Point(54, 95), cv::Point(293, 296), cv::Scalar(255, 255, 255), -1);
    // draw a blue circle
    cv::circle(test_image, cv::Point(300, 500), 100, cv::Scalar(255, 0, 0), -1);
    // draw a green line
    cv::line(test_image, cv::Point(600, 400), cv::Point(400, 500), cv::Scalar(0, 255, 0), 5);
    // draw a red filled polygon
    std::vector<cv::Point> hexagon = {cv::Point(605, 605), cv::Point(705, 605), cv::Point(755, 655), cv::Point(705, 705), cv::Point(605, 705), cv::Point(555, 655)};
    cv::fillConvexPoly(test_image, hexagon, cv::Scalar(0, 0, 255));

    // Create a Frame object
    cv::Mat input_image;
    cv::cvtColor(test_image, input_image, cv::COLOR_BGR2GRAY);
    Frame frame(input_image);


    // search for the edge point every edge_find_window_size pixels and 30 degrees
    std::vector<EdgePoint> edge_points;
    EdgePointFinder edge_point_finder;

    for (int i = edge_find_window_size; i < window_size - edge_find_window_size; i += edge_find_window_size) {
        for (int j = edge_find_window_size; j < window_size - edge_find_window_size; j += edge_find_window_size) {
            for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 6) {
                bool result = false;
                EdgePoint edge_point = edge_point_finder.find_key_edge_point(frame,
                                                                             cv::Point(i + edge_find_window_size / 2, j + edge_find_window_size / 2),
                                                                             angle,
                                                                             edge_find_window_size,
                                                                             result);

                if (result) {
                    edge_points.push_back(edge_point);
                }
            }
        }
    }

    std::cout << "Number of edge points: " << edge_points.size() << std::endl;

    // Create a DebugView
    DebugView debug_view(input_image);
    debug_view.drawEdgePoints(edge_points);

    // Save the image
    cv::imwrite(RESULT_IMAGE_PATH "edge_points_with_test_img.jpg", debug_view.getDebugImage());
}
