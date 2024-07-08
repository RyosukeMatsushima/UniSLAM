#ifndef DOUBLE_SQUARES_SPACE_HPP
#define DOUBLE_SQUARES_SPACE_HPP

#include "polygons_space.hpp"

class DoubleSquaresSpace
{
public:
    DoubleSquaresSpace();

    cv::Mat getImage(const cv::InputArray &rvec,
                     const cv::InputArray &tvec) const;

private:
    PolygonsSpace polygons_space;
};

#endif // DOUBLE_SQUARES_SPACE_HPP

