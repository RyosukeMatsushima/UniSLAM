
#include "frame.h"

Frame::Frame(const cv::Mat& input_img)
{
    cv::Mat gray_img = confirmGrayImage(input_img);
    edge_gausian_img_ = differentialOfGaussianImage(gray_img);
}

cv::Mat Frame::confirmGrayImage(const cv::Mat& input_img)
{
    // Check if the input image is empty
    if (input_img.empty()) {
        throw std::invalid_argument("Input image is empty!");
    }

    // Convert the input image to grayscale if it's not already
    cv::Mat gray_img;
    if (input_img.channels() > 1) {
        cv::cvtColor(input_img, gray_img, cv::COLOR_BGR2GRAY);
    } else {
        gray_img = input_img.clone();
    }
    return gray_img;
}

cv::Mat Frame::differentialOfGaussianImage(const cv::Mat& gray_img) {

    // Calculate the edge image with differential Gaussian filter
    cv::Mat smoothed_img_with_small_kernel;
    cv::GaussianBlur(gray_img, smoothed_img_with_small_kernel, cv::Size(5, 5), 1.0);

    cv::Mat smoothed_img_with_large_kernel;
    cv::GaussianBlur(gray_img, smoothed_img_with_large_kernel, cv::Size(9, 9), 2.0);

    cv::Mat differential_gausian_img;
    cv::subtract(smoothed_img_with_small_kernel, smoothed_img_with_large_kernel, differential_gausian_img);
    return differential_gausian_img;
}
