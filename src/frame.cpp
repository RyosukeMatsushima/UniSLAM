
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

    calculateGradientImages(gray_img_, gradient_x_img_, gradient_y_img_);

    calculateGradientAngleImage(gradient_x_img_, gradient_y_img_, gradient_angle_img_);

    calculateDiscreteAngleEdgeIntensity(laplacian_img_, gradient_angle_img_, discrete_angle_edge_intensity_);
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

std::vector<cv::Mat> Frame::getDiscreteAngleEdgeIntensity() const
{
    return discrete_angle_edge_intensity_;
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
    cv::Mat magnitude_img;
    cv::cartToPolar(gradient_x_img, gradient_y_img, magnitude_img, gradient_angle_img, true);
}

void Frame::calculateDiscreteAngleEdgeIntensity(const cv::Mat& laplacian_img,
                                                const cv::Mat& gradient_angle_img,
                                                std::vector<cv::Mat>& discrete_angle_edge_intensity)
{
    int num_discrete_angles = int(2 * M_PI / MAX_ANGLE_DIFF);

    // normalize the gradient angle
    cv::Mat normalized_gradient_angle;
    cv::normalize(gradient_angle_img, normalized_gradient_angle, 0, num_discrete_angles - 1, cv::NORM_MINMAX, CV_8U);

    // calculate the edge intensity for each discrete angle
    for (int i = 0; i < num_discrete_angles; i++) {
        cv::Mat mask = (normalized_gradient_angle == i);
        cv::Mat edge_intensity;

        // format to CV_32F
        mask.convertTo(mask, CV_32F);
        // normalize the mask
        cv::normalize(mask, mask, 0.0f, 1.0f, cv::NORM_MINMAX, CV_32F);

        cv::multiply(laplacian_img, mask, edge_intensity);
        discrete_angle_edge_intensity.push_back(edge_intensity);
    }
}


