#ifndef FRAME_H
#define FRAME_H

#include <opencv2/opencv.hpp>

#include "edge.h"

#define EDGE_THRESHOLD 0.0007f
#define GAUSIAN_KERNEL_SIZE_SMALL 1.0f
#define GAUSIAN_KERNEL_SIZE_LARGE 20.0f


class Frame {
public:
    Frame(const cv::Mat& input_img);

    cv::Mat getGrayImage() const;

    cv::Mat getEdgeGausianImage() const;

    cv::Mat getLaplacianImage() const;

    bool getKeyEdgePoints(const int window_size,
                          const int num_keypoints,
                          std::vector<EdgePoint>& key_edge_points);

    bool getMatchedEdgePoints(const EdgePoint& key_edge_point,
                              const int window_size,
                              std::vector<EdgePoint>& matched_edge_points);

private:
    cv::Mat gray_img_;

    cv::Mat edge_gausian_img_;

    cv::Mat laplacian_img_;

    cv::Mat confirmGrayImage(const cv::Mat& input_img);

    cv::Mat differentialOfGaussianImage(const cv::Mat& input_img);

    cv::Mat calculateLaplacianImage(const cv::Mat& dog_img);
};

#endif // FRAME_H

