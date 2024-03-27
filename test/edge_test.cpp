#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "edge.h"

class EdgeGenerator {
public:
    EdgeGenerator(cv::Vec2f direction, cv::Point2f start_point, float magnitude) {
        this->direction = direction;
        this->current_point = start_point;
        this->magnitude = magnitude;
    };

    void GenerateNextPoint(cv::Point2f& point, cv::Vec2f& gradient) {
        current_point.x += direction[0] * magnitude; current_point.y += direction[1] * magnitude;
        // gradient is 90 degree rotated direction
        gradient = cv::Vec2f(-direction[1], direction[0]);
        point = current_point;
        return;
    };

    cv::Point2f GetCurrentPoint() {
        return current_point;
    };

private:
    cv::Vec2f direction;
    cv::Point2f current_point;
    float magnitude;
};

TEST(EdegeTest, TestNormalCase) {
    Edge edge;

    // add 10 edge points
    EdgeGenerator generator(cv::Vec2f(1.0f, 0.0f), cv::Point2f(10.0f, 20.0f), 1.0f);

    for (int i = 0; i < 10; i++) {
        cv::Point2f point;
        cv::Vec2f gradient;
        generator.GenerateNextPoint(point, gradient);
        EXPECT_TRUE(edge.addPoint(point, gradient));
    }

    // check edge size
    EXPECT_EQ(edge.size(), 10);
}

TEST(EdgeTest, TestJoinEdge) {
    Edge edge1;
    Edge* edge2 = new Edge();

    // add 10 edge points
    EdgeGenerator generator(cv::Vec2f(1.0f, 0.0f), cv::Point2f(10.0f, 20.0f), 1.0f);

    for (int i = 0; i < 10; i++) {
        cv::Point2f point;
        cv::Vec2f gradient;
        generator.GenerateNextPoint(point, gradient);
        EXPECT_TRUE(edge1.addPoint(point, gradient));
    }

    for (int i = 0; i < 10; i++) {
        cv::Point2f point;
        cv::Vec2f gradient;
        generator.GenerateNextPoint(point, gradient);
        EXPECT_TRUE(edge2->addPoint(point, gradient));
    }

    // join edge2 to edge1
    EXPECT_TRUE(edge1.joinEdge(*edge2));

    // delete edge2
    delete edge2;
}

