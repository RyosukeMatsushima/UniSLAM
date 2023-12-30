#ifndef FRAME_H
#define FRAME_H

#include <opencv2/opencv.hpp>

class Frame {
public:
    Frame(const cv::Mat& input_img);

private:
    cv::Mat edge_gausian_img_;

    cv::Mat confirmGrayImage(const cv::Mat& input_img);

    cv::Mat differentialOfGaussianImage(const cv::Mat& input_img);

};

#endif // FRAME_H

