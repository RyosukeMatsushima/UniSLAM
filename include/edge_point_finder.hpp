#ifndef EDGE_POINT_FINDER_HPP
#define EDGE_POINT_FINDER_HPP

#include <opencv2/opencv.hpp>

#include "frame.h"
#include "edge.h"
#include "edge_point_check.hpp"

class EdgePointFinder {
public:
    EdgePointFinder();

    EdgePoint find_key_edge_point(const Frame frame,
                                  const cv::Point& entry_point,
                                  const float angle,
                                  const int window_size,
                                  bool& result) const;

    cv::Point find_max_intensity_point(const cv::Mat& intensity_map) const;

    cv::Point point_in_frame(const cv::Point point_in_window,
                             const cv::Point window_center) const;
};

#endif // EDGE_POINT_FINDER_HPP
