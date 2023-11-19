#include "ImageProcessor.h"

ImageProcessor::ImageProcessor(const std::string& filePath, double threshold) {
    image = loadImage(filePath);
    computeGradients(image);
    displayGradientMagnitude();
    convertVectorsToUnitVectors(threshold);
    displayGradientDirection();
    saveVectorAngles();
    displayImages();
}

cv::Mat ImageProcessor::loadImage(const std::string& filePath) {
    cv::Mat loadedImage = cv::imread(filePath, cv::IMREAD_GRAYSCALE);

    if (loadedImage.channels() == 3) {
        cv::cvtColor(loadedImage, loadedImage, cv::COLOR_BGR2GRAY);
    }

    return loadedImage;
}

void ImageProcessor::computeGradients(const cv::Mat& inputImage) {
    cv::Sobel(inputImage, gradient_x, CV_64F, 1, 0, 3);
    cv::Sobel(inputImage, gradient_y, CV_64F, 0, 1, 3);
    cv::magnitude(gradient_x, gradient_y, gradient_magnitude);
}

void ImageProcessor::displayGradientMagnitude() {
    cv::imshow("Gradient Magnitude", gradient_magnitude / 255.0);
    cv::waitKey(0);
}

void ImageProcessor::convertVectorsToUnitVectors(double threshold) {
    cv::phase(gradient_x, gradient_y, gradient_direction, true);
    cv::threshold(gradient_magnitude, gradient_magnitude, threshold, 255, cv::THRESH_BINARY);
    gradient_direction.setTo(0, gradient_magnitude < threshold);
}

void ImageProcessor::displayGradientDirection() {
    cv::imshow("Gradient Direction", gradient_direction / (2 * CV_PI));
    cv::waitKey(0);
}

void ImageProcessor::saveVectorAngles() {
    gradient_direction.convertTo(angleImage, CV_8U, 180 / CV_PI);
}

void ImageProcessor::displayImages() {
    cv::imshow("Original Image", image);
    cv::imshow("Gradient Direction (in degrees)", angleImage);
    cv::waitKey(0);
}

