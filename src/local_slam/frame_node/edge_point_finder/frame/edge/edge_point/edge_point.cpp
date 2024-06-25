
#include "edge_point.hpp"


EdgePoint::EdgePoint(const cv::Point2f& point,
                     const cv::Vec2f& gradient)
{
    this->point = point;
    this->magnitude = cv::norm(gradient);

    // angle range is 0 to 2pi
    this->angle = toAngle(gradient);
    this->gradient = gradient;

    // direction is 90 degree rotated normalised gradient
    this->direction[0] = -gradient[1] / magnitude;
    this->direction[1] = gradient[0] / magnitude;
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


float EdgePoint::AngleDiff(const float angle1, const float angle2)
{
    float diff = angle2 - angle1;
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

// TODO: rename to convertToAngle
float EdgePoint::toAngle(const cv::Vec2f& vec) const
{
    float angle = atan2(vec[1], vec[0]);
    if (angle < 0)
    {
        angle += 2 * M_PI;
    }
    return angle;
}

// TODO: rename to convertToVec
cv::Vec2f EdgePoint::toUnitVec(const float angle) const
{
    return cv::Vec2f(cos(angle), sin(angle));
}

float EdgePoint::distanceTo(const EdgePoint& target) const
{
    cv::Vec2f diff = target.point - point;
    // dot product of direction vector and diff vector
    return abs(diff[0] * gradient[0] + diff[1] * gradient[1]) / magnitude;
}

float EdgePoint::angleTo(const EdgePoint& target) const
{
    return AngleDiff(angle, target.angle);
}
