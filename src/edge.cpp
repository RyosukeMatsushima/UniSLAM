
#include "edge.h"

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

// TODO: rename this function to points()
std::vector<EdgePoint> Edge::getEdgePoints()
{
    std::vector<EdgePoint> edge_points;
    for (auto& point : edge_points_)
    {
        edge_points.push_back(point);
    }
    return edge_points;
}

int Edge::size() const
{
    return edge_points_.size();
}

