#ifndef CAMERA_MODEL_HPP
#define CAMERA_MODEL_HPP

#include <opencv2/opencv.hpp>

#include "edge_node.hpp"
#include "edge_point.hpp"

class CameraModel {
public:
    CameraModel(const cv::Mat& camera_matrix,
                const cv::Size& img_size);

    EdgeNode getEdgeNode(const EdgePoint& edge_point) const;

private:
    cv::Mat camera_matrix;
    cv::Size img_size;
};

#endif // CAMERA_MODEL_HPP
