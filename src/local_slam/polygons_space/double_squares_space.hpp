#ifndef DOUBLE_SQUARES_SPACE_HPP
#define DOUBLE_SQUARES_SPACE_HPP

#include "polygons_space.hpp"

class DoubleSquaresSpace
{
public:
    DoubleSquaresSpace();

    cv::Mat getImage(const cv::InputArray &rvec,
                     const cv::InputArray &tvec) const;

    cv::Size getImageSize() const { return image_size; }

    cv::Mat getCameraMatrix() const { return K; }

private:
    PolygonsSpace polygons_space;

    cv::Size image_size = cv::Size(500, 500);

    cv::Mat K = (cv::Mat_<double>(3, 3) << 100, 0, image_size.width / 2,
                                           0, 100, image_size.height / 2,
                                           0, 0, 1);
};

#endif // DOUBLE_SQUARES_SPACE_HPP

