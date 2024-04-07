
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

std::vector<EdgePoint> Frame::getKeyEdgePoints()
{
    double min_intensity, max_intensity;
    cv::Point min_loc, max_loc;
    cv::minMaxLoc(laplacian_img_, &min_intensity, &max_intensity, &min_loc, &max_loc);

    float min_intensity_threshold = max_intensity * MIN_INTENSITY_THRESHOLD;

    std::vector<EdgePoint> key_edge_points;

    for (cv::Mat& edge_intensity : discrete_angle_edge_intensity_) {
        std::vector<EdgePoint> edge_points = getHighIntensityEdgePoints(min_intensity_threshold);
        key_edge_points.insert(key_edge_points.end(), edge_points.begin(), edge_points.end());
    }
    return key_edge_points;
}

bool Frame::getMatchedEdgePoints(const EdgePoint& key_edge_point,
                                 const int window_size,
                                 std::vector<EdgePoint>& matched_edge_points)
{
    return false;
}

cv::Vec2f Frame::getGradient(const cv::Point2f& point)
{
    // read the gradient from the gradient images
    float gradient_x = gradient_x_img_.at<float>(point);
    float gradient_y = gradient_y_img_.at<float>(point);

    return cv::Vec2f(gradient_x, gradient_y);
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

std::vector<EdgePoint> Frame::getHighIntensityEdgePoints(const float min_intensity_threshold)
{
    std::vector<EdgePoint> edge_points;

    int block_width = laplacian_img_.cols / BLOCK_NUM_X;
    int block_height = laplacian_img_.rows / BLOCK_NUM_Y;
    
    for (int i = 0; i < BLOCK_NUM_X; i++) {
        for (int j = 0; j < BLOCK_NUM_Y; j++) {
            cv::Rect block_rect(i * block_width, j * block_height, block_width, block_height);
            cv::Mat block = laplacian_img_(block_rect);

            // get max intensity point
            double min_intensity, max_intensity;
            cv::Point min_loc, max_loc;

            int max_try_count = 3;
            for (int try_count = 0; try_count < max_try_count; try_count++) {

                cv::minMaxLoc(block, &min_intensity, &max_intensity, &min_loc, &max_loc);
    
                if (max_intensity < min_intensity_threshold) {
                    continue;
                }

                // ignore edge pixels of the block since they could be not local maximum
                if (max_loc.x == 0 || max_loc.y == 0 || max_loc.x == block_width - 1 || max_loc.y == block_height - 1) {
                    // set the intensity of the max_loc to 0
                    block.at<float>(max_loc) = 0;
                    continue;
                }

                cv::Point2f max_loc_in_img = cv::Point2f(max_loc.x + i * block_width, max_loc.y + j * block_height);
                EdgePoint edge_point = EdgePoint(max_loc_in_img, getGradient(max_loc_in_img));
    
                edge_points.push_back(edge_point);

                break;
            }
        }
    }

    return edge_points;
}


