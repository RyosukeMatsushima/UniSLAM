
#include "edge_point_finder.hpp"

EdgePointFinder::EdgePointFinder()
{
}

EdgePoint EdgePointFinder::find_key_edge_point(const Frame frame,
                                               const cv::Point& entry_point,
                                               const float gradient_angle,
                                               const int window_size,
                                               bool& result) const
{
    // entry_point should have buffer from the frame edge

    cv::Mat intensity_block = frame.discreteAngleEdgeIntensity.getBlockIntensity(entry_point, window_size, gradient_angle, 0);

    bool is_valid;
    cv::Point max_intensity_point_in_window = find_max_intensity_point(intensity_block, is_valid);

    if (!is_valid) {
        result = false;
        return EdgePoint(cv::Point(0, 0), cv::Vec2f(0, 0));
    }

    cv::Point max_intensity_point = cv::Point(max_intensity_point_in_window.x + entry_point.x - window_size / 2,
                                              max_intensity_point_in_window.y + entry_point.y - window_size / 2);

    EdgePoint edgePoint(max_intensity_point,
                        frame.getGradient(cv::Point2f(max_intensity_point.x, max_intensity_point.y)));

    // check if the edge point is valid
    cv::Mat edge_centered_intensity_map = frame.discreteAngleEdgeIntensity.getBlockIntensity(edgePoint.point,
                      window_size,
                      edgePoint.angle,
                      1);

    result = EdgePointCheck(edge_centered_intensity_map,
                            edgePoint.direction).is_valid();

    return edgePoint;
}

cv::Point EdgePointFinder::find_max_intensity_point(const cv::Mat& intensity_map,
                                                    bool& is_valid) const
{
    cv::Point min_intensity_point, max_intensity_point;
    double min_intensity, max_intensity;
    cv::minMaxLoc(intensity_map, &min_intensity, &max_intensity, &min_intensity_point, &max_intensity_point);

    is_valid = max_intensity > 0;

    return max_intensity_point;
}


