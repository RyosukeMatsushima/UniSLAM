
#include "frame.h"
Frame::Frame(const cv::Mat& input_img)
{
    gray_img_ = confirmGrayImage(input_img);

    // check type of the image
    if (gray_img_.type() != CV_32F) {
        throw std::invalid_argument("Input image is not CV_32F!");
    }

    edge_gausian_img_ = differentialOfGaussianImage(gray_img_);

    laplacian_img_ = calculateLaplacianImage(edge_gausian_img_);
}

cv::Mat Frame::getGrayImage() const
{
    return gray_img_;
}

cv::Mat Frame::getEdgeGausianImage() const
{
    return edge_gausian_img_;
}

cv::Mat Frame::getLaplacianImage() const
{
    return laplacian_img_;
}

bool Frame::getKeyEdgePoints(const int window_size,
                             const int num_keypoints,
                             std::vector<EdgePoint>& key_edge_points)
{
    return false;
}

bool Frame::getMatchedEdgePoints(const EdgePoint& key_edge_point,
                                 const int window_size,
                                 std::vector<EdgePoint>& matched_edge_points)
{
    return false;
}

cv::Mat Frame::confirmGrayImage(const cv::Mat& input_img)
{
    printf("input_img.type() = %d\n", input_img.type());
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

    // Change type of the image to float
    gray_img.convertTo(gray_img, CV_32F);

    // Normalize the image
    cv::normalize(gray_img, gray_img, 0.0f, 100.0f, cv::NORM_MINMAX, CV_32F);

    return gray_img;
}

cv::Mat Frame::differentialOfGaussianImage(const cv::Mat& gray_img) {

    // Calculate the edge image with differential Gaussian filter
    cv::Mat smoothed_img_with_small_kernel;
    cv::GaussianBlur(gray_img, smoothed_img_with_small_kernel, cv::Size(9, 9), GAUSIAN_KERNEL_SIZE_SMALL);

    cv::Mat smoothed_img_with_large_kernel;
    cv::GaussianBlur(gray_img, smoothed_img_with_large_kernel, cv::Size(9, 9), GAUSIAN_KERNEL_SIZE_LARGE);

    cv::Mat differential_gausian_img = smoothed_img_with_small_kernel - smoothed_img_with_large_kernel;

    return differential_gausian_img;
}

cv::Mat Frame::calculateLaplacianImage(const cv::Mat& dog_img) {
    cv::Mat laplacian_img;
    cv::Mat abs_dog_img = cv::abs(dog_img);
    cv::Laplacian(abs_dog_img, laplacian_img, CV_32F, 3);
    cv::normalize(laplacian_img, laplacian_img, 0.0f, 255.0f, cv::NORM_MINMAX, CV_32F);
    return laplacian_img;
}
