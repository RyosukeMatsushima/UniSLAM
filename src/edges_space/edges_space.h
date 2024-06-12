#ifndef EDGES_SPACE_H
#define EDGES_SPACE_H

#include <opencv2/opencv.hpp>

struct Edge
{
    cv::Point3f start_point;
    cv::Point3f end_point;
};

class EdgesSpace
{
public:
    EdgesSpace();

    cv::Mat getImage(const cv::InputArray &rvec, const cv::InputArray &tvec, const cv::InputArray &camera_matrix, const cv::InputArray &dist_coeffs, const cv::Size &image_size);

    void setEdge(const cv::Point3f &start_point, const cv::Point3f &end_point);

private:

    std::vector<Edge> edges;
};

#endif // EDGES_SPACE_H
