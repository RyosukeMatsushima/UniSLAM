
#include "frame.h"

Frame::Frame(const cv::Mat& input_img,
             const float angle_resolution)
{
    gray_img_ = confirmGrayImage(input_img);

    // check type of the image
    if (gray_img_.type() != CV_32F) {
        throw std::invalid_argument("Input image is not CV_32F!");
    }

    edge_gausian_img_ = differentialOfGaussianImage(gray_img_);

    laplacian_img_ = calculateLaplacianImage(edge_gausian_img_);

    calculateGradientImages(gray_img_, gradient_x_img_, gradient_y_img_);

    calculateGradientAngleImage(gradient_x_img_, gradient_y_img_, gradient_angle_img_);

    discreteAngleEdgeIntensity.setIntensityMap(laplacian_img_, gradient_angle_img_, angle_resolution);
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

cv::Mat Frame::getGradientXImage() const
{
    return gradient_x_img_;
}

cv::Mat Frame::getGradientYImage() const
{
    return gradient_y_img_;
}

cv::Mat Frame::getGradientAngleImage() const
{
    return gradient_angle_img_;
}

bool Frame::getMatchedEdgePoints(const EdgePoint& key_edge_point,
                                 const int window_size,
                                 std::vector<EdgePoint>& matched_edge_points)
{
    return false;
}

cv::Vec2f Frame::getGradient(const cv::Point2f& point) const
{
    // read the gradient from the gradient images
    float gradient_x = gradient_x_img_.at<float>(point);
    float gradient_y = gradient_y_img_.at<float>(point);

    return cv::Vec2f(gradient_x, gradient_y);
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

    // Change type of the image to float
    gray_img.convertTo(gray_img, CV_32F);

    // Normalize the image
    cv::normalize(gray_img, gray_img, 0.0f, 100.0f, cv::NORM_MINMAX, CV_32F);

    // Blur the image
    cv::GaussianBlur(gray_img, gray_img, cv::Size(9, 9), GAUSIAN_KERNEL_SIZE_SMALL);

    return gray_img;
}

cv::Mat Frame::differentialOfGaussianImage(const cv::Mat& gray_img) {

    // gray_img is already blurred

    cv::Mat smoothed_img_with_large_kernel;
    cv::GaussianBlur(gray_img, smoothed_img_with_large_kernel, cv::Size(9, 9), GAUSIAN_KERNEL_SIZE_LARGE);

    cv::Mat differential_gausian_img = gray_img - smoothed_img_with_large_kernel;

    return differential_gausian_img;
}

cv::Mat Frame::calculateLaplacianImage(const cv::Mat& dog_img) {
    cv::Mat laplacian_img;
    cv::Mat abs_dog_img = cv::abs(dog_img);
    cv::Laplacian(abs_dog_img, laplacian_img, CV_32F, 3);
    return laplacian_img;
}

void Frame::calculateGradientImages(const cv::Mat& input_img,
                                    cv::Mat& gradient_x_img,
                                    cv::Mat& gradient_y_img)
{
    cv::Mat gradient_x;
    cv::Mat gradient_y;
    cv::Sobel(input_img, gradient_x, CV_32F, 1, 0, 3);
    cv::Sobel(input_img, gradient_y, CV_32F, 0, 1, 3);

    gradient_x_img = gradient_x;
    gradient_y_img = gradient_y;
}

void Frame::calculateGradientAngleImage(const cv::Mat& gradient_x_img,
                                        const cv::Mat& gradient_y_img,
                                        cv::Mat& gradient_angle_img)
{
    cv::Mat magnitude_img; // not used

    // angle is in the range of [0, 2 * M_PI]
    cv::cartToPolar(gradient_x_img, gradient_y_img, magnitude_img, gradient_angle_img, false);

    // set invalid angle -1 if gradient x and y are both 0
    cv::Mat invalid_value_mask = (gradient_x_img == 0) & (gradient_y_img == 0);
    gradient_angle_img.setTo(-1, invalid_value_mask);
}

