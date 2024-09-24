#include "edge_distribution.hpp"
#include <stdexcept>

EdgeDistribution::EdgeDistribution(cv::Size imgSize, cv::Size windowSize) 
    : imgSize(imgSize), windowSize(windowSize) {
    cv::Size edgeMapSize(imgSize.width / windowSize.width, imgSize.height / windowSize.height);
    edgeMap = cv::Mat::zeros(edgeMapSize, CV_32SC1);
}

void EdgeDistribution::add_edge_point(cv::Point point) {
    if (point.x < 0 || point.x >= imgSize.width || point.y < 0 || point.y >= imgSize.height) {
        throw std::invalid_argument("Point is out of image bounds");
    }
    cv::Point window(point.x / windowSize.width, point.y / windowSize.height);

    // Increment the number of edge points in the window and around it
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            cv::Point windowAround = window + cv::Point(i, j);
            if (windowAround.x >= 0 && windowAround.x < edgeMap.cols && windowAround.y >= 0 && windowAround.y < edgeMap.rows) {
                edgeMap.at<int>(windowAround) += 1;
            }
        }
    }
}

void EdgeDistribution::remove_edge_point(cv::Point point) {
    if (point.x < 0 || point.x >= imgSize.width || point.y < 0 || point.y >= imgSize.height) {
        throw std::invalid_argument("Point is out of image bounds");
    }
    cv::Point window(point.x / windowSize.width, point.y / windowSize.height);

    // Decrement the number of edge points in the window and around it
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            cv::Point windowAround = window + cv::Point(i, j);
            if (windowAround.x >= 0 && windowAround.x < edgeMap.cols && windowAround.y >= 0 && windowAround.y < edgeMap.rows) {
                edgeMap.at<int>(windowAround) -= 1;
            }
        }
    }
}

std::vector<cv::Point> EdgeDistribution::get_empty_windows() const {
    std::vector<cv::Point> emptyWindows;

    // find empty elements in edgeMap = 0
    for (int i = 0; i < edgeMap.rows; i++) {
        for (int j = 0; j < edgeMap.cols; j++) {
            if (edgeMap.at<int>(i, j) == 0) {
                emptyWindows.push_back(cv::Point(j * windowSize.width, i * windowSize.height));
            }
        }
    }
    return emptyWindows;
}

int EdgeDistribution::edge_num(cv::Point point) {
    if (point.x < 0 || point.x >= imgSize.width || point.y < 0 || point.y >= imgSize.height) {
        throw std::invalid_argument("Point is out of image bounds");
    }
    cv::Point window(point.x / windowSize.width, point.y / windowSize.height);
    return edgeMap.at<int>(window);
}

EdgeDistribution& EdgeDistribution::operator=(const EdgeDistribution& other_edge_distribution) {
    imgSize = other_edge_distribution.imgSize;
    windowSize = other_edge_distribution.windowSize;
    edgeMap = other_edge_distribution.edgeMap.clone();
    return *this;
}

void EdgeDistribution::clear() {
    edgeMap = cv::Mat::zeros(edgeMap.size(), edgeMap.type());
}

