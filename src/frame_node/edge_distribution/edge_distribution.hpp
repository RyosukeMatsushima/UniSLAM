#ifndef EDGE_DISTRIBUTION_HPP
#define EDGE_DISTRIBUTION_HPP

#include <opencv2/opencv.hpp>

class EdgeDistribution {
public:
    EdgeDistribution(cv::Size imgSize, cv::Size windowSize);
    void add_edge_point(cv::Point point);
    void remove_edge_point(cv::Point point);
    std::vector<cv::Point> get_empty_windows() const;
    int edge_num(cv::Point point);
    EdgeDistribution& operator=(const EdgeDistribution& other_edge_distribution);
    void clear();

private:
    cv::Size imgSize;
    cv::Size windowSize;
    cv::Mat edgeMap;
};

#endif // EDGE_DISTRIBUTION_HPP
