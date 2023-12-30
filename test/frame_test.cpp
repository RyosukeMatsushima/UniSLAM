#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "frame.h"

#define TEST_IMAGE_PATH "../test/data/sequence_01/imgs/00000.jpg"

#define RESULT_IMAGE_PATH "./result/"

// Test the Frame class
TEST(FrameTest, TestFrame) {

    // check test image exists
    std::ifstream f(TEST_IMAGE_PATH);
    if (!f.good()) {
        std::cout << "Test image not found: " << TEST_IMAGE_PATH << std::endl;

        // print current directory
        char cwd[1024];
        if (getcwd(cwd, sizeof(cwd)) != NULL) {
            printf("Current working dir: %s\n", cwd);
        } else {
            perror("getcwd() error");
        }

        return;
    }
    // Load an image (replace "your_image_path" with the actual path to an image)
    cv::Mat input_image = cv::imread(TEST_IMAGE_PATH);

    // Create a Frame object
    Frame frame(input_image);
}

// Test for confirmGrayImage function
TEST(FrameTest, TestConfirmGrayImage) {
    // Load an image (replace "your_image_path" with the actual path to an image)
    cv::Mat input_image = cv::imread(TEST_IMAGE_PATH);

    // Create a Frame object
    Frame frame(input_image);

    // Show image
    cv::imshow("Input Image", input_image);

    // Save image
    cv::imwrite(RESULT_IMAGE_PATH "input_image.jpg", input_image);
}

