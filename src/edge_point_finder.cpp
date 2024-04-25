
#include "edge_point_finder.hpp"

EdgePointFinder::EdgePointFinder()
{
}

EdgePoint EdgePointFinder::find_key_edge_point(const Frame frame,
                                               const cv::Point& entry_point,
                                               const float angle,
                                               const int window_size,
                                               bool& result) const
{
    cv::Mat intensity_block = frame.discreteAngleEdgeIntensity.getBlockIntensity(entry_point, window_size, angle, 0);

    cv::Point max_intensity_point = point_in_frame(find_max_intensity_point(intensity_block), entry_point);

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

cv::Point EdgePointFinder::find_max_intensity_point(const cv::Mat& intensity_map) const
{
    cv::Point max_intensity_point;
    cv::minMaxLoc(intensity_map, NULL, NULL, NULL, &max_intensity_point);

    return max_intensity_point;
}

cv::Point EdgePointFinder::point_in_frame(const cv::Point point_in_window,
                         const cv::Point window_center) const
{
    cv::Point point_in_frame;
    point_in_frame.x = point_in_window.x + window_center.x;
    point_in_frame.y = point_in_window.y + window_center.y;
    return point_in_frame;
}

