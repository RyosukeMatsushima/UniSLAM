
#include "polygons_space.hpp"

PolygonsSpace::PolygonsSpace()
{
}

// return vector field as cv::Mat(image_size, CV_8UC3)
cv::Mat PolygonsSpace::getImage(const cv::InputArray &rvec,
                                const cv::InputArray &tvec,
                                const cv::InputArray &camera_matrix,
                                const cv::InputArray &dist_coeffs, 
                                const cv::Size &image_size)
{
    cv::Mat image = cv::Mat::zeros(image_size, CV_8UC3);

    for (const auto &polygon: polygons)
    {
        // project 3D points to 2D image
        std::vector<cv::Point2f> projected_points;
        cv::projectPoints(polygon.points, rvec, tvec, camera_matrix, dist_coeffs, projected_points);

        std::vector<cv::Point> image_points;
        for (const auto &point: projected_points)
        {
            image_points.push_back(cv::Point(static_cast<int>(point.x), static_cast<int>(point.y)));
        }

        // draw filled polygon
        cv::fillConvexPoly(image, image_points, polygon.color, cv::LINE_AA);
    }

    return image;
}

void PolygonsSpace::setPolygon(const std::vector<cv::Point3f> &points,
                            const cv::Scalar &color)
{
    Polygon polygon;
    polygon.points = points;
    polygon.color = color;
    
    polygons.push_back(polygon);
}
