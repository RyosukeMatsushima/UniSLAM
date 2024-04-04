#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "frame.h"
#include "debug_view.hpp"

#define TEST_IMAGE_PATH "../test/data/sequence_01/imgs/00000.jpg"

#define RESULT_IMAGE_PATH "./result/"

bool checkImageExist(const std::string &image_path) {
    std::ifstream f(image_path);
    if (!f.good()) {
        std::cout << "Test image not found: " << image_path << std::endl;

        // print current directory
        char cwd[1024];
        if (getcwd(cwd, sizeof(cwd)) != NULL) {
            printf("Current working dir: %s\n", cwd);
        } else {
            perror("getcwd() error");
        }
    }

    return f.good();
}

// Test the Frame class
TEST(FrameTest, TestFrame) {

    if (!checkImageExist(TEST_IMAGE_PATH)) {
        return;
    }

    // Load an image (replace "your_image_path" with the actual path to an image)
    cv::Mat input_image = cv::imread(TEST_IMAGE_PATH);

    // Create a Frame object
    Frame frame(input_image);

    cv::imwrite(RESULT_IMAGE_PATH "gray_img.jpg", frame.getGrayImage());

    cv::imwrite(RESULT_IMAGE_PATH "edge_gausian_img.jpg", frame.getEdgeGausianImage());

    cv::Mat laplacian_img = frame.getLaplacianImage();
    // normalize the image
    cv::normalize(laplacian_img, laplacian_img, 0, 255, cv::NORM_MINMAX, CV_8U);

    cv::imwrite(RESULT_IMAGE_PATH "laplacian_img.jpg", laplacian_img);

    cv::imwrite(RESULT_IMAGE_PATH "gradient_x_img.jpg", frame.getGradientXImage());

    cv::imwrite(RESULT_IMAGE_PATH "gradient_y_img.jpg", frame.getGradientYImage());

    cv::imwrite(RESULT_IMAGE_PATH "gradient_angle_img.jpg", frame.getGradientAngleImage());

    std::vector<cv::Mat> discrete_angle_edge_intensity = frame.getDiscreteAngleEdgeIntensity();
    cv::Mat sum_discrete_angle_edge_intensity = cv::Mat::zeros(discrete_angle_edge_intensity[0].size(), CV_32F);
    for (int i = 0; i < discrete_angle_edge_intensity.size(); i++) {
        sum_discrete_angle_edge_intensity += discrete_angle_edge_intensity[i];
        // normalize the image
        cv::normalize(discrete_angle_edge_intensity[i], discrete_angle_edge_intensity[i], 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::imwrite(RESULT_IMAGE_PATH "discrete_angle_edge_intensity_" + std::to_string(i) + ".jpg", discrete_angle_edge_intensity[i]);
    }

    // compare sum_discrete_angle_edge_intensity with laplacian_img
    cv::Mat diff;
    cv::absdiff(sum_discrete_angle_edge_intensity, frame.getLaplacianImage(), diff);
    cv::imwrite(RESULT_IMAGE_PATH "diff_sum_discrete_angle_edge_intensity_laplacian_img.jpg", diff);
    // diff should be all zeros
    cv::Scalar s = cv::sum(diff);
    ASSERT_EQ(s[0], 0.0);

    std::vector<EdgePoint> key_edge_points = frame.getKeyEdgePoints();
    std::cout << "size of key_edge_points: " << key_edge_points.size() << std::endl;

    // Create a DebugView object
    DebugView debug_view(frame.getGrayImage());
    debug_view.drawEdgePoints(key_edge_points);
    cv::imwrite(RESULT_IMAGE_PATH "key_edge_points.jpg", debug_view.getDebugImage());

    // print edge_gausian_img
    cv::imshow("edge_gausian_img", frame.getEdgeGausianImage());
}

// Test for confirmGrayImage function
TEST(FrameTest, TestConfirmGrayImage) {
    
    if (!checkImageExist(TEST_IMAGE_PATH)) {
        return;
    }

    // Load an image (replace "your_image_path" with the actual path to an image)
    cv::Mat input_image = cv::imread(TEST_IMAGE_PATH);

    // Create a Frame object
    Frame frame(input_image);

    // Show image
    cv::imshow("Input Image", input_image);

    // Save image
    cv::imwrite(RESULT_IMAGE_PATH "input_image.jpg", input_image);
}
