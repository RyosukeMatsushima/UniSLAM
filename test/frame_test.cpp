#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "frame.h"

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

    cv::imwrite(RESULT_IMAGE_PATH "laplacian_img.jpg", frame.getLaplacianImage());

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
