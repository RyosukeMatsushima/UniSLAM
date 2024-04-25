#ifndef DISCRETE_ANGLE_EDGE_INTENSITY_HPP
#define DISCRETE_ANGLE_EDGE_INTENSITY_HPP

#include <opencv2/opencv.hpp>

class DiscreteAngleEdgeIntensity {
public:
    DiscreteAngleEdgeIntensity();

    void setIntensityMap(const cv::Mat& intensity_map,
                    const cv::Mat& gradient_angle_map,
                    const float angle_resolution);

    cv::Mat getBlockIntensity(const cv::Point& center_point,
                              const int block_size,
                              const float angle,
                              int angle_range) const;

    int discretizeAngle(const float angle) const;

private:
    float angle_resolution_ = 0;

    std::vector<cv::Mat> discrete_angle_edge_intensity_;
};

#endif // DISCRETE_ANGLE_EDGE_INTENSITY_HPP
