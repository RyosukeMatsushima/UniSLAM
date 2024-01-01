#ifndef FRAME_H
#define FRAME_H

#include <opencv2/opencv.hpp>

#define EDGE_THRESHOLD 0.04f
#define GAUSIAN_KERNEL_SIZE_SMALL 10.0f
#define GAUSIAN_KERNEL_SIZE_LARGE 20.0f


class Frame {
public:
    Frame(const cv::Mat& input_img);

    cv::Mat getGrayImage() const;

    cv::Mat getEdgeGausianImage() const;

private:
    cv::Mat gray_img_;

    cv::Mat edge_gausian_img_;

    cv::Mat confirmGrayImage(const cv::Mat& input_img);

    cv::Mat differentialOfGaussianImage(const cv::Mat& input_img);

};

#endif // FRAME_H

