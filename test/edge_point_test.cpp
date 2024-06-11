#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>

#include "edge_point.hpp"

TEST(EgdePointTest, AngleDiff)
{
    float angle1 = 0;
    float angle2 = 0;
    EXPECT_FLOAT_EQ(EdgePoint::AngleDiff(angle1, angle2), 0);

    angle1 = M_PI / 2;
    angle2 = 3 * M_PI / 2;
    EXPECT_FLOAT_EQ(EdgePoint::AngleDiff(angle1, angle2), M_PI);

    angle1 = 3 * M_PI / 2;
    angle2 = M_PI / 2;
    EXPECT_FLOAT_EQ(EdgePoint::AngleDiff(angle1, angle2), -M_PI);
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
    cv::Point2f point(1, 1);
    cv::Vec2f gradient(2, 2);
    EdgePoint edge_point(point, gradient);

    cv::Point2f new_point(2, 2);
    cv::Vec2f new_gradient(2, 2);
    EdgePoint new_edge_point(new_point, new_gradient);

    EXPECT_TRUE(edge_point.isContinuous(new_edge_point, true));
}

TEST(EgdePointTest, GetDiffToMatchedPoint)
{
    // case 1
    cv::Point2f point(1, 1);
    cv::Vec2f gradient(1, 0);
    EdgePoint edge_point(point, gradient);

    cv::Point2f new_point(2, 2);
    cv::Vec2f new_gradient(0, 1);
    EdgePoint new_edge_point(new_point, new_gradient);

    // distance between two points connected the direction of the edge
    EXPECT_NEAR(edge_point.distanceTo(new_edge_point), 1, 1e-6);

    EXPECT_NEAR(edge_point.angleTo(new_edge_point), M_PI / 2, 1e-6);

    // case 2
    point = cv::Point2f(1, 0);
    gradient = cv::Vec2f(1, 0);
    edge_point = EdgePoint(point, gradient);

    new_point = cv::Point2f(2, 0);
    new_gradient = cv::Vec2f(1, 0);
    new_edge_point = EdgePoint(new_point, new_gradient);

    // distance between two points connected the direction of the edge
    EXPECT_NEAR(edge_point.distanceTo(new_edge_point), 1, 1e-6);
    EXPECT_NEAR(edge_point.angleTo(new_edge_point), 0, 1e-6);

    // case 3
    point = cv::Point2f(1, 0);
    gradient = cv::Vec2f(0, 1);
    edge_point = EdgePoint(point, gradient);

    new_point = cv::Point2f(2, 0);
    new_gradient = cv::Vec2f(-1, 0);
    new_edge_point = EdgePoint(new_point, new_gradient);

    // distance between two points connected the direction of the edge
    EXPECT_NEAR(edge_point.distanceTo(new_edge_point), 0, 1e-6);
    EXPECT_NEAR(edge_point.angleTo(new_edge_point), M_PI / 2, 1e-6);
}

