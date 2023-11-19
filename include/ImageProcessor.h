#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <opencv2/opencv.hpp>

class ImageProcessor {
public:
    ImageProcessor(const std::string& filePath, double threshold = 50.0);

private:
    cv::Mat loadImage(const std::string& filePath);
    void computeGradients(const cv::Mat& image);
    void displayGradientMagnitude();
    void convertVectorsToUnitVectors(double threshold);
    void displayGradientDirection();
    void saveVectorAngles();
    void displayImages();

    cv::Mat image;
    cv::Mat gradient_x, gradient_y, gradient_magnitude, gradient_direction, angleImage;
};

#endif // IMAGEPROCESSOR_H

