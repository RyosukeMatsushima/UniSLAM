

#include "edges_space.h"

EdgesSpace::EdgesSpace()
{
}

// return vector field as cv::Mat(image_size, CV_32FC2)
cv::Mat EdgesSpace::getImage(const cv::InputArray &rvec, const cv::InputArray &tvec, const cv::InputArray &camera_matrix, const cv::InputArray &dist_coeffs, const cv::Size &image_size)
{
    cv::Mat image = cv::Mat::zeros(image_size, CV_32FC2);

    for (const auto &edge: edges)
    {
        cv::Point3f start_point = edge.start_point;
        cv::Point3f end_point = edge.end_point;

        std::vector<cv::Point3f> object_points;
        std::vector<cv::Point2f> image_points;

        object_points.push_back(cv::Point3f(start_point.x, start_point.y, start_point.z));
        object_points.push_back(cv::Point3f(end_point.x, end_point.y, end_point.z));

        cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, image_points);

        cv::Point2f line_direction_normalized = (image_points[1] - image_points[0]) / cv::norm(image_points[1] - image_points[0]);

        cv::line(image, image_points[0], image_points[1], cv::Scalar(line_direction_normalized.x, line_direction_normalized.y), 1);
    }

    return image;
}

void EdgesSpace::setEdge(const cv::Point3f &start_point, const cv::Point3f &end_point)
{
    Edge edge;
    edge.start_point = start_point;
    edge.end_point = end_point;

    edges.push_back(edge);
}
