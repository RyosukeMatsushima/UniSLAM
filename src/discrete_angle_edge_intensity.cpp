
#include "discrete_angle_edge_intensity.hpp"

DiscreteAngleEdgeIntensity::DiscreteAngleEdgeIntensity() {
}

// angle should be in the range of [0, 2 * M_PI]
void DiscreteAngleEdgeIntensity::setIntensityMap(const cv::Mat& intensity_map,
                                                 const cv::Mat& gradient_angle_map,
                                                 const float angle_resolution) {
    angle_resolution_ = angle_resolution;

    // calculate the discrete angle edge intensity
    int num_discrete_angles = int(2 * M_PI / angle_resolution_);

    cv::Mat shifted_angle_map = gradient_angle_map + angle_resolution_ / 2;

    for (int i = 0; i < num_discrete_angles; i++) {
        float angle = i * angle_resolution_;
        cv::Mat mask = (shifted_angle_map >= angle) & (shifted_angle_map < angle + angle_resolution_);
        cv::Mat intensity = cv::Mat::zeros(intensity_map.size(), CV_32F);
        intensity_map.copyTo(intensity, mask);
        discrete_angle_edge_intensity_.push_back(intensity);
    }

}

cv::Mat DiscreteAngleEdgeIntensity::getBlockIntensity(const cv::Point& center_point,
                                                      const int block_size,
                                                      const float gradient_angle,
                                                      int angle_range) const {
    int angle_index = discretizeAngle(gradient_angle);

    // angle range should be more than zero or zero
    if ( angle_range < 0 ) {
        angle_range = 0;
    }

    cv::Rect block_rect(center_point.x - block_size / 2, center_point.y - block_size / 2, block_size, block_size);

    cv::Mat block_intensity = cv::Mat::zeros(block_size, block_size, CV_32F);

    for (int i = -angle_range; i <= angle_range; i++) {
        int current_angle_index = (angle_index + i);
        if (current_angle_index < 0) {
            current_angle_index += discrete_angle_edge_intensity_.size();
        }
        if (current_angle_index >= discrete_angle_edge_intensity_.size()) {
            current_angle_index -= discrete_angle_edge_intensity_.size();
        }
        cv::Mat intensity = discrete_angle_edge_intensity_[current_angle_index](block_rect);
        block_intensity += intensity;
    }

    return block_intensity;
}

int DiscreteAngleEdgeIntensity::discretizeAngle(const float angle) const {
    int num_discrete_angles = discrete_angle_edge_intensity_.size();

    float shifted_angle = angle + angle_resolution_ / 2;

    return int( shifted_angle / angle_resolution_ ) % num_discrete_angles;
}

