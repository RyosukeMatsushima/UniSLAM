#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "edge_point_finder.hpp"
#include "edge.h"
#include "frame.h"
#include "debug_view.hpp"

#define TEST_IMAGE_PATH PROJECT_SOURCE_DIR "/test/data/00000.jpg"
#define RESULT_IMAGE_PATH PROJECT_SOURCE_DIR "/test/result/"

std::vector<EdgePoint> searchEveryArea(Frame frame, int edge_find_window_size, float angle) {
    std::vector<EdgePoint> edge_points;
    EdgePointFinder edge_point_finder;
    auto gray_image = frame.getGrayImage();
    
    for (int i = edge_find_window_size; i < gray_image.cols - edge_find_window_size; i += edge_find_window_size) {
        for (int j = edge_find_window_size; j < gray_image.rows - edge_find_window_size; j += edge_find_window_size) {
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
    return edge_points;
}

std::string getFileName(const std::string& name, float angle) {
    return RESULT_IMAGE_PATH + name + "_search_angle_" + std::to_string(static_cast<int>(angle * 180 / M_PI)) + ".jpg";
}

bool checkEdgePointAngle(const std::vector<EdgePoint>& edge_points, float angle, float threshold) {
    for (const auto& edge_point : edge_points) {
        float diff = std::abs(edge_point.angle - angle);
        if (diff + threshold > 2 * M_PI) {
            diff -= 2 * M_PI;
        }
        if (diff > threshold) {
            std::cout << "Angle: " << edge_point.angle << " Expected: " << angle << " Threshold: " << threshold << std::endl;
            std::cout << "gradient: " << edge_point.gradient << std::endl;
            std::cout << "point: " << edge_point.point << std::endl;
            return false;
        }
    }
    return true;
}

cv::Mat createTestImageWithLine(int image_width, float line_angle, cv::Point offset = cv::Point(0, 0)) {
    cv::Mat input_image = cv::Mat::zeros(image_width, image_width, CV_32F);
    int x = static_cast<int>(image_width / 2 * cos(line_angle));
    int y = static_cast<int>(image_width / 2 * sin(line_angle));
    cv::Point start(image_width / 2 - x, image_width / 2 - y);
    cv::Point end(image_width / 2 + x, image_width / 2 + y);
    cv::line(input_image, start + offset, end + offset, cv::Scalar(255), 5);
    return input_image;
}

void testEdgePointFinderWithImage(const cv::Mat& input_image, int image_width, float gradient_angle, float angle_resolution, int edge_find_window_size) {
    Frame frame(input_image, 0.2f); // TODO: change to angle_resolution
    EdgePointFinder edge_point_finder;
    bool result = false;
    EdgePoint edge_point = edge_point_finder.find_key_edge_point(frame,
                                                                 cv::Point(image_width / 2, image_width / 2),
                                                                 gradient_angle,
                                                                 edge_find_window_size,
                                                                 result);
    DebugView debug_view(input_image);
    debug_view.drawEdgePoints({edge_point});
    std::string img_name = std::to_string(static_cast<int>(gradient_angle * 180 / M_PI)) + "_line.jpg";
    std::string img_path = RESULT_IMAGE_PATH;
    cv::imwrite(img_path + "edge_point_with_" + img_name, debug_view.getDebugImage());

    ASSERT_TRUE(result);
    ASSERT_TRUE(checkEdgePointAngle({edge_point}, gradient_angle, angle_resolution));

    float offset_size = edge_find_window_size / 2;
    cv::Point offset = cv::Point(static_cast<int>(offset_size * cos(gradient_angle)), static_cast<int>(offset_size * sin(gradient_angle)));
    cv::Mat input_image_with_offset = createTestImageWithLine(image_width, gradient_angle + M_PI / 2, offset);
    Frame frame_with_offset(input_image_with_offset, 0.2f); // TODO: change to angle_resolution
    EdgePoint edge_point_with_offset = edge_point_finder.find_key_edge_point(frame_with_offset,
                                                                             edge_point.point,
                                                                             edge_point.angle,
                                                                             edge_find_window_size,
                                                                             result);
    EXPECT_TRUE(result);
    EXPECT_TRUE(checkEdgePointAngle({edge_point_with_offset}, gradient_angle, angle_resolution));
    
    DebugView debug_view_with_offset(input_image_with_offset);
    debug_view_with_offset.drawEdgePoints({edge_point_with_offset});
    std::string img_name_with_offset = std::to_string(static_cast<int>(gradient_angle * 180 / M_PI)) + "_line_with_offset.jpg";
    cv::imwrite(img_path + "edge_point_with_" + img_name_with_offset, debug_view_with_offset.getDebugImage());
}

TEST(EdgePointFinderTests, createTestImageWithLine) {
    int image_width = 200;
    float line_angle = M_PI / 4;
    cv::Mat input_image = createTestImageWithLine(image_width, line_angle);
    cv::Mat input_image_with_offset = createTestImageWithLine(image_width, line_angle, cv::Point(15, 0));
    cv::imwrite(RESULT_IMAGE_PATH "line.jpg", input_image);
    cv::imwrite(RESULT_IMAGE_PATH "line_with_offset.jpg", input_image_with_offset);
}

TEST(EdgePointFinderTests, TestEdgePointFinderWithLine) {
    int image_width = 200;
    int edge_find_window_size = 30;
    float angle_resolution = M_PI / 12;

    for (float gradient_angle = 0.0f; gradient_angle < 2 * M_PI; gradient_angle += angle_resolution) {
        float line_angle = gradient_angle + M_PI / 2;
        cv::Mat input_image = createTestImageWithLine(image_width, line_angle);
        testEdgePointFinderWithImage(input_image, image_width, gradient_angle, angle_resolution, edge_find_window_size);

        std::vector<EdgePoint> edge_points = searchEveryArea(Frame(input_image, 0.2f), edge_find_window_size, gradient_angle);
        ASSERT_TRUE(checkEdgePointAngle(edge_points, gradient_angle, angle_resolution));

        DebugView debug_view(input_image);
        debug_view.drawEdgePoints(edge_points);
        cv::imwrite(getFileName("lint_test_img", line_angle), debug_view.getDebugImage());
    }
}

void drawShapes(cv::Mat& test_image) {
    cv::rectangle(test_image, cv::Point(54, 95), cv::Point(293, 296), cv::Scalar(255, 255, 255), -1);
    cv::circle(test_image, cv::Point(300, 500), 100, cv::Scalar(255, 0, 0), -1);
    cv::line(test_image, cv::Point(600, 400), cv::Point(400, 500), cv::Scalar(0, 255, 0), 5);
    std::vector<cv::Point> hexagon = {cv::Point(605, 605), cv::Point(705, 605), cv::Point(755, 655), cv::Point(705, 705), cv::Point(605, 705), cv::Point(555, 655)};
    cv::fillConvexPoly(test_image, hexagon, cv::Scalar(0, 0, 255));
}

void testEdgePointFinderWithShapes(float angle_resolution, int edge_find_window_size) {
    int window_size = 1000;
    cv::Mat test_image = cv::Mat::zeros(window_size, window_size, CV_8UC3);
    drawShapes(test_image);
    cv::Mat input_image;
    cv::cvtColor(test_image, input_image, cv::COLOR_BGR2GRAY);
    Frame frame(input_image, 0.2f); // TODO: change to angle_resolution

    for (double angle = 0; angle < 2 * M_PI; angle += angle_resolution) {
        std::vector<EdgePoint> edge_points = searchEveryArea(frame, edge_find_window_size, angle);
        ASSERT_TRUE(checkEdgePointAngle(edge_points, angle, angle_resolution));

        DebugView debug_view(input_image);
        debug_view.drawEdgePoints(edge_points);
        cv::imwrite(getFileName("edge_points_with_test_img", angle), debug_view.getDebugImage());
    }
}

TEST(EdgePointFinderTests, TestEdgePointFinderWithShapes) {
    testEdgePointFinderWithShapes(M_PI / 12, 30);
}

TEST(EdgePointFinderTests, TestEdgePointFinderWithCircle) {
    int window_size = 1000;
    int edge_find_window_size = 30;
    float angle_resolution = M_PI / 6;
    cv::Mat test_image = cv::Mat::zeros(window_size, window_size, CV_8UC3);
    cv::circle(test_image, cv::Point(500, 500), 400, cv::Scalar(255, 0, 0), -1);

    cv::Mat input_image;
    cv::cvtColor(test_image, input_image, cv::COLOR_BGR2GRAY);
    Frame frame(input_image, 0.2f); // TODO: change to angle_resolution

    for (double angle = 0; angle < 2 * M_PI; angle += angle_resolution) {
        std::vector<EdgePoint> edge_points = searchEveryArea(frame, edge_find_window_size, angle);
        DebugView debug_view(input_image);
        debug_view.drawEdgePoints(edge_points);
        cv::imwrite(getFileName("edge_points_with_circle_img", angle), debug_view.getDebugImage());
        ASSERT_TRUE(checkEdgePointAngle(edge_points, angle, angle_resolution));
    }
}

TEST(EdgePointFinderTests, TestEdgePointFinderWithCameraImg) {
    cv::Mat test_image = cv::imread(TEST_IMAGE_PATH);
    cv::Mat input_image;
    cv::cvtColor(test_image, input_image, cv::COLOR_BGR2GRAY);
    float angle_resolution = M_PI / 12;
    Frame frame(test_image, 0.2f); // TODO: change to angle_resolution
    int edge_find_window_size = 40;

    for (double angle = 0; angle < 2 * M_PI; angle += angle_resolution) {
        std::vector<EdgePoint> edge_points = searchEveryArea(frame, edge_find_window_size, angle);
        ASSERT_TRUE(checkEdgePointAngle(edge_points, angle, angle_resolution));

        DebugView debug_view(input_image);
        debug_view.drawEdgePoints(edge_points);
        cv::imwrite(getFileName("edge_points_with_camera_img", angle), debug_view.getDebugImage());
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

