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


