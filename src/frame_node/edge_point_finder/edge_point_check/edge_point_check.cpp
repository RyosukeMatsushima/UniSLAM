
#include "edge_point_check.hpp"

EdgePointCheck::EdgePointCheck(const cv::Mat& edge_centered_intensity_map,
                               const cv::Vec2f& edge_direction)
    : edge_centered_intensity_map_(edge_centered_intensity_map),
      edge_direction_(edge_direction)
{
    // check_area_radius_ is set to 1/2 of the smaller image size
    check_area_radius_ = std::min(edge_centered_intensity_map.cols, edge_centered_intensity_map.rows) / 2;
}

bool EdgePointCheck::is_valid() const
{
    cv::Mat edge_centered_intensity_map = edge_centered_intensity_map_;
    // remove unused area
    cv::Mat masked_intensity_map = remove_unused_area(edge_centered_intensity_map);

    // threshold is center value * EDGE_THRESHOLD_RATIO
    float threshold = masked_intensity_map.at<float>(masked_intensity_map.rows / 2, masked_intensity_map.cols / 2) * EDGE_THRESHOLD_RATIO;

    // count over threshold
    int total_over_threshold_count = over_threshold_count(masked_intensity_map, threshold);

    // count over threshold in desired edge area
    cv::Mat desired_edge_area = get_desired_edge_area(masked_intensity_map);
    int over_threshold_count_in_edge = over_threshold_count(desired_edge_area, threshold);

    // check edge purity
    float edge_purity = float(over_threshold_count_in_edge) / float(total_over_threshold_count);

    int edge_pixel_count_threshold = std::min(masked_intensity_map.cols, masked_intensity_map.rows) * MIN_EDGE_PIXEL_COUNT_RATE;

    return (edge_purity > MIN_EDGE_PURITY_THRESHOLD) && (over_threshold_count_in_edge > edge_pixel_count_threshold);
}

int EdgePointCheck::over_threshold_count(const cv::Mat& intensity_map, float threshold) const
{
    cv::Mat over_threshold_map;
    cv::threshold(intensity_map, over_threshold_map, threshold, 1, cv::THRESH_BINARY);

    return cv::countNonZero(over_threshold_map);
}

cv::Mat EdgePointCheck::remove_unused_area(cv::Mat& intensity_map) const
{
    // set zero to unused area
    // unused area is defined as outof circle with radius of smoller image size

    cv::Mat mask = cv::Mat::zeros(intensity_map.size(), CV_8UC1);
    cv::circle(mask, cv::Point(intensity_map.cols / 2, intensity_map.rows / 2), check_area_radius_, cv::Scalar(255), -1);

    cv::Mat masked_intensity_map;

    intensity_map.copyTo(masked_intensity_map, mask);

    return masked_intensity_map;
}

cv::Mat EdgePointCheck::get_desired_edge_area(cv::Mat& intensity_map) const
{
    // set zero except desired edge area

    cv::Mat mask = cv::Mat::zeros(intensity_map.size(), CV_8UC1);

    cv::Point center(intensity_map.cols / 2, intensity_map.rows / 2);
    cv::Point start_point(center.x + edge_direction_[0] * check_area_radius_,
                          center.y + edge_direction_[1] * check_area_radius_);
    cv::Point edge_point(center.x - edge_direction_[0] * check_area_radius_,
                         center.y - edge_direction_[1] * check_area_radius_);

    cv::line(mask, start_point, edge_point, cv::Scalar(225), EDGE_WIDTH);

    cv::Mat masked_intensity_map;
    intensity_map.copyTo(masked_intensity_map, mask);

    return masked_intensity_map;
}
