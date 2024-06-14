#ifndef FRAME_H
#define FRAME_H

#include <opencv2/opencv.hpp>

#include "edge.h"
#include "discrete_angle_edge_intensity.hpp"

#define GAUSIAN_KERNEL_SIZE_SMALL 1.0f
#define GAUSIAN_KERNEL_SIZE_LARGE 20.0f

class Frame {
public:
    Frame(const cv::Mat& input_img,
          const float angle_resolution);

    cv::Mat getGrayImage() const;

    cv::Mat getEdgeGausianImage() const;

    cv::Mat getLaplacianImage() const;

    cv::Mat getGradientXImage() const;

    cv::Mat getGradientYImage() const;

    cv::Mat getGradientAngleImage() const;

    bool getMatchedEdgePoints(const EdgePoint& key_edge_point,
                              const int window_size,
                              std::vector<EdgePoint>& matched_edge_points);

    cv::Vec2f getGradient(const cv::Point2f& point) const;

    // TODO: recheck this usage
    DiscreteAngleEdgeIntensity discreteAngleEdgeIntensity;

private:
    cv::Mat gray_img_;

    cv::Mat edge_gausian_img_;

    cv::Mat laplacian_img_;

    cv::Mat gradient_x_img_;

    cv::Mat gradient_y_img_;

    cv::Mat gradient_angle_img_;

    cv::Mat confirmGrayImage(const cv::Mat& input_img);

    cv::Mat differentialOfGaussianImage(const cv::Mat& input_img);

    cv::Mat calculateLaplacianImage(const cv::Mat& dog_img);

    void calculateGradientImages(const cv::Mat& input_img,
                                 cv::Mat& gradient_x_img,
                                 cv::Mat& gradient_y_img);

    void calculateGradientAngleImage(const cv::Mat& gradient_x_img,
                                     const cv::Mat& gradient_y_img,
                                     cv::Mat& gradient_angle_img);
};

#endif // FRAME_H

