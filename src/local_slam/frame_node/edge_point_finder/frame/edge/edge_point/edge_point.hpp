#ifndef EDGE_POINT_HPP
#define EDGE_POINT_HPP

#include <opencv2/opencv.hpp>

#define CONTINUOUS_ANGLE_THRESHOLD 0.1f
#define CONTINUOUS_DISTANCE_THRESHOLD 1.7f

struct EdgePoint {
    int edge_id{-1};  // -1 means not assigned
    cv::Point2f point;
    float angle;
    float magnitude;
    cv::Vec2f gradient;
    cv::Vec2f direction;

    EdgePoint(const cv::Point2f& point,
              const cv::Vec2f& gradient);

    bool isContinuous(const EdgePoint& new_point,
                      bool is_to_back) const;

    static float AngleDiff(float angle1, float angle2);

    float toAngle(const cv::Vec2f& vec) const;

    cv::Vec2f toUnitVec(float angle) const;

    float distanceTo(const EdgePoint& target) const;

    float angleTo(const EdgePoint& target) const;
};

#endif // EDGE_POINT_HPP
