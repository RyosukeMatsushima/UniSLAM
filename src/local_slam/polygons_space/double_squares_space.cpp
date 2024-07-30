#include "double_squares_space.hpp"

DoubleSquaresSpace::DoubleSquaresSpace()
{
    float square_size = 0.05;
    float base_z = 0.05;
    // draw square in x-z plane
    cv::Point3f p1(-square_size, square_size, base_z);
    cv::Point3f p2(square_size, square_size, base_z);
    cv::Point3f p3(square_size, square_size, base_z + square_size);
    cv::Point3f p4(-square_size, square_size, base_z + square_size);

    cv::Point3f p5(-square_size, -square_size, base_z);
    cv::Point3f p6(square_size, -square_size, base_z);
    cv::Point3f p7(square_size, -square_size, base_z + square_size);
    cv::Point3f p8(-square_size, -square_size, base_z + square_size);

    square_size = 0.4;
    base_z = 1.0;
    cv::Point3f p9(-square_size, -square_size, base_z);
    cv::Point3f p10(square_size, -square_size, base_z);
    cv::Point3f p11(square_size, square_size, base_z);
    cv::Point3f p12(-square_size, square_size, base_z);

    std::vector<cv::Point3f> polygon1 = {p1, p2, p3, p4};
    std::vector<cv::Point3f> polygon2 = {p5, p6, p7, p8};
    std::vector<cv::Point3f> polygon3 = {p9, p10, p11, p12};

    polygons_space.setPolygon(polygon1, cv::Scalar(0, 0, 255));
    polygons_space.setPolygon(polygon2, cv::Scalar(0, 255, 0));
    polygons_space.setPolygon(polygon3, cv::Scalar(255, 0, 0));
}

cv::Mat DoubleSquaresSpace::getImage(const cv::InputArray &rvec,
                                     const cv::InputArray &tvec) const
{
    return polygons_space.getImage(rvec, tvec, K, image_size);
}

