#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>

#include "edge_point_finder.hpp"
#include "edge.h"
#include "frame.h"
#include "debug_view.hpp"

#define RESULT_IMAGE_PATH "./result/edge_point_finder/"


TEST(EdgePointFinderTest, TestEdgePointFinder) {
    int image_width = 200;
    int edge_find_window_size = 30;
    cv::Mat input_image = cv::Mat::zeros(image_width, image_width, CV_32F);

    // Draw a line through the center of the image with a angle of 45 degrees
    cv::line(input_image, cv::Point(0, 0), cv::Point(image_width, image_width), cv::Scalar(255), 3);

    Frame frame(input_image);

    EdgePointFinder edge_point_finder;

    bool result = false;
    EdgePoint edge_point = edge_point_finder.find_key_edge_point(frame,
                                                                 cv::Point(image_width / 2, image_width / 2),
                                                                 2 * M_PI - M_PI / 4,
                                                                 edge_find_window_size,
                                                                 result);

    // Save the images
    cv::imwrite(RESULT_IMAGE_PATH "laplacian.jpg", frame.getLaplacianImage());
    cv::imwrite(RESULT_IMAGE_PATH "gradient_angle.jpg", frame.getGradientAngleImage());

    // print the edge point
    std::cout << "edge_point: " << edge_point.point << std::endl;
    // result should be true
    ASSERT_TRUE(result);

    // Create a DebugView
    DebugView debug_view(input_image);
    debug_view.drawEdgePoints({edge_point});

    // Save the image
    cv::imwrite(RESULT_IMAGE_PATH "edge_point_with_45_line.jpg", debug_view.getDebugImage());

}
