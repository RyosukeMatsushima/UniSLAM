
#include "edge.h"

float AngleDiff(const float angle1, const float angle2)
{
    float diff = angle1 - angle2;
    if (diff > M_PI)
    {
        diff -= 2 * M_PI;
    }
    else if (diff < -M_PI)
    {
        diff += 2 * M_PI;
    }
    return diff;
}

EdgePoint::EdgePoint(const cv::Point2f& point,
                     const cv::Vec2f& gradient)
{
    this->point = point;
    this->magnitude = cv::norm(gradient);
    this->angle = atan2(gradient[1], gradient[0]);
}

bool EdgePoint::isContinuous(const EdgePoint& new_point,
                             bool is_to_back) const
{
    cv::Vec2f point_diff = new_point.point - point;

    // check each points are neighbors
    if (cv::norm(point_diff) > CONTINUOUS_DISTANCE_THRESHOLD)
    {
        return false;
    }

    // check angle is smaller than threshold
    float gradient_angle_diff = AngleDiff(angle, new_point.angle);
    if (fabs(gradient_angle_diff) > CONTINUOUS_ANGLE_THRESHOLD)
    {
        return false;
    }

    // check new_point is to back or front
    float angle_to_new_point = atan2(point_diff[1], point_diff[0]);

    if (is_to_back)
    {
        if (AngleDiff(angle, angle_to_new_point) > 0)
        {
            return false;
        }
    }
    else
    {
        if (AngleDiff(angle, angle_to_new_point) < 0)
        {
            return false;
        }
    }
    return true;
}

Edge::Edge()
{
}

bool Edge::addPoint(const cv::Point2f& point,
                    const cv::Vec2f& gradient)
{
    EdgePoint edge_point(point, gradient);

    // if edge_points_ is empty
    if (edge_points_.size() == 0)
    {
        edge_points_.push_back(edge_point);
        return true;
    }

    // add new point to back if it is continuous with last point
    if (edge_points_.back().isContinuous(edge_point, true))
    {
        edge_points_.push_back(edge_point);
        return true;
    }

    // add new point to front if it is continuous with first point
    if (edge_points_.front().isContinuous(edge_point, false))
    {
        edge_points_.insert(edge_points_.begin(), edge_point);
        return true;
    }
    return false;
}

bool Edge::joinEdge(Edge& joining_edge)
{
    std::vector<EdgePoint> joining_edge_points = joining_edge.getEdgePoints();

    // if edge_points_ is empty
    if (edge_points_.size() == 0)
    {
        edge_points_ = joining_edge_points;
        return true;
    }

    // if joining_edge_points is empty
    if (joining_edge_points.size() == 0)
    {
        return true;
    }

    // check if joining_edge_points is continuous with edge_points_
    if (edge_points_.back().isContinuous(joining_edge_points.front(), true))
    {
        edge_points_.insert(edge_points_.end(),
                            joining_edge_points.begin(),
                            joining_edge_points.end());
        return true;
    }

    // check if joining_edge_points is continuous with edge_points_
    if (edge_points_.front().isContinuous(joining_edge_points.back(), false))
    {
        edge_points_.insert(edge_points_.begin(),
                            joining_edge_points.rbegin(),
                            joining_edge_points.rend());
        return true;
    }
    return false;
}

std::vector<EdgePoint> Edge::getEdgePoints()
{
    std::vector<EdgePoint> edge_points;
    for (auto& point : edge_points_)
    {
        edge_points.push_back(point);
    }
    return edge_points;
}

