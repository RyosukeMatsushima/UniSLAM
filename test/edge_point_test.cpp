#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>

#include "edge_point.hpp"

TEST(EgdePointTest, AngleDiff)
{
    float angle1 = 0;
    float angle2 = 0;
    EXPECT_FLOAT_EQ(EdgePoint::AngleDiff(angle1, angle2), 0);

    angle1 = 0;
    angle2 = M_PI;
    EXPECT_FLOAT_EQ(EdgePoint::AngleDiff(angle1, angle2), M_PI);

    angle1 = M_PI;
    angle2 = 0;
    EXPECT_FLOAT_EQ(EdgePoint::AngleDiff(angle1, angle2), -M_PI);

    angle1 = M_PI / 2;
    angle2 = 3 * M_PI / 2;
    EXPECT_FLOAT_EQ(EdgePoint::AngleDiff(angle1, angle2), -M_PI);

    angle1 = 3 * M_PI / 2;
    angle2 = M_PI / 2;
    EXPECT_FLOAT_EQ(EdgePoint::AngleDiff(angle1, angle2), M_PI);
}

TEST(EgdePointTest, EdgePoint)
{
    cv::Point2f point(1, 2);
    cv::Vec2f gradient(3, 4);
    EdgePoint edge_point(point, gradient);

    EXPECT_EQ(edge_point.point, point);
    EXPECT_FLOAT_EQ(edge_point.magnitude, 5);
    EXPECT_FLOAT_EQ(edge_point.angle, atan2(4, 3));
    EXPECT_EQ(edge_point.gradient, gradient);
    EXPECT_FLOAT_EQ(edge_point.direction[0], -4 / 5.0);
    EXPECT_FLOAT_EQ(edge_point.direction[1], 3 / 5.0);
}

TEST(EgdePointTest, IsContinuous)
{
    cv::Point2f point(1, 2);
    cv::Vec2f gradient(3, 4);
    EdgePoint edge_point(point, gradient);

    cv::Point2f new_point(1.5, 2.5);
    cv::Vec2f new_gradient(3.5, 4.5);
    EdgePoint new_edge_point(new_point, new_gradient);

    EXPECT_TRUE(edge_point.isContinuous(new_edge_point, true));
    EXPECT_FALSE(edge_point.isContinuous(new_edge_point, false));
}

TEST(EgdePointTest, GetDiffToMatchedPoint)
{
    cv::Point2f point(1, 2);
    cv::Vec2f gradient(3, 4);
    EdgePoint edge_point(point, gradient);

    cv::Point2f new_point(1.5, 2.5);
    cv::Vec2f new_gradient(3.5, 4.5);
    EdgePoint new_edge_point(new_point, new_gradient);

    EXPECT_FLOAT_EQ(edge_point.distanceTo(new_edge_point), cv::norm(new_point - point));
    EXPECT_FLOAT_EQ(edge_point.angleTo(new_edge_point), atan2(new_point.y - point.y, new_point.x - point.x));
}

