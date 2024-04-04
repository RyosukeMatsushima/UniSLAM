#ifndef EDGE_H
#define EDGE_H

#include <opencv2/opencv.hpp>

#define CONTINUOUS_ANGLE_THRESHOLD 0.1f
#define CONTINUOUS_DISTANCE_THRESHOLD 1.7f

struct EdgePoint {
    cv::Point2f point;
    float angle;
    float magnitude;
    cv::Vec2f gradient;

    EdgePoint(const cv::Point2f& point,
              const cv::Vec2f& gradient);

    bool isContinuous(const EdgePoint& new_point,
                      bool is_to_back) const;
};

class Edge {
public:
    Edge();

    bool addPoint(const cv::Point2f& point,
                  const cv::Vec2f& gradient);

    std::vector<EdgePoint> getEdgePoints() const;

    bool joinEdge(Edge& joining_edge);

    std::vector<EdgePoint> getEdgePoints();

    int size() const;

private:
    std::vector<EdgePoint> edge_points_;
};

#endif // EDGE_H
