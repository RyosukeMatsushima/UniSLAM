#ifndef FRAME_H
#define FRAME_H

#include <opencv2/opencv.hpp>

#define EDGE_THRESHOLD 0.8f

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

