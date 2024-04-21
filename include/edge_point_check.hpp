#ifndef EDGE_POINT_CHECK_HPP
#define EDGE_POINT_CHECK_HPP

#define EDGE_WIDTH 3
#define EDGE_THRESHOLD_RATIO 0.5
#define MIN_EDGE_PURITY_THRESHOLD 0.6f

#include <opencv2/opencv.hpp>

class EdgePointCheck {
public:
    EdgePointCheck(const cv::Mat& edge_centered_intensity_map,
                   const cv::Vec2f& edge_direction);

    bool is_valid() const;

    int over_threshold_count(const cv::Mat& intensity_map, float threshold) const;

    cv::Mat remove_unused_area(cv::Mat& intensity_map) const;

    cv::Mat get_desired_edge_area(cv::Mat& intensity_map) const;

private:
    cv::Mat edge_centered_intensity_map_;
    cv::Vec2f edge_direction_;

    int check_area_radius_;
};

#endif // EDGE_POINT_CHECK_HPP


    

