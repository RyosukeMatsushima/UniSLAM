#include "double_squares_space.hpp"

DoubleSquaresSpace::DoubleSquaresSpace()
{
    // draw square in x-z plane
    cv::Point3f p1(-1, 1, 1);
    cv::Point3f p2(1, 1, 1);
    cv::Point3f p3(1, 1, 2);
    cv::Point3f p4(-1, 1, 2);

    cv::Point3f p5(-1, -1, 1);
    cv::Point3f p6(1, -1, 1);
    cv::Point3f p7(1, -1, 2);
    cv::Point3f p8(-1, -1, 2);

    std::vector<cv::Point3f> polygon1 = {p1, p2, p3, p4};
    std::vector<cv::Point3f> polygon2 = {p5, p6, p7, p8};

    polygons_space.setPolygon(polygon1, cv::Scalar(0, 0, 255));
    polygons_space.setPolygon(polygon2, cv::Scalar(0, 255, 0));
}

cv::Mat DoubleSquaresSpace::getImage(const cv::InputArray &rvec,
                                     const cv::InputArray &tvec) const
{
    cv::Size image_size(500, 500);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 100, 0, image_size.width / 2,
                                           0, 100, image_size.height / 2,
                                           0, 0, 1);

    return polygons_space.getImage(rvec, tvec, K, image_size);
}

