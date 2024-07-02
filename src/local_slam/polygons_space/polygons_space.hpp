#ifndef POLYGONS_SPACE_HPP
#define POLYGONS_SPACE_HPP

#include <opencv2/opencv.hpp>

struct Polygon
{
    std::vector<cv::Point3f> points;
    cv::Scalar color;
};

class PolygonsSpace
{
public:
    PolygonsSpace();

    cv::Mat getImage(const cv::InputArray &rvec,
                     const cv::InputArray &tvec,
                     const cv::InputArray &camera_matrix,
                     const cv::Size &image_size);

    void setPolygon(const std::vector<cv::Point3f> &points,
                    const cv::Scalar &color);

private:

    std::vector<Polygon> polygons;
};

#endif // POLYGONS_SPACE_HPP
