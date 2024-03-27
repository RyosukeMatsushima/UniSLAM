
#include "edge_detector.hpp"

EdgeDetector::EdgeDetector(const cv::Mat& laplacian_img,
                           const cv::Mat& gradient_x,
                           const cv::Mat& gradient_y)
{
    // check size of the input images are the same
    if (laplacian_img.size() != gradient_x.size() ||
        laplacian_img.size() != gradient_y.size())
    {
        throw std::invalid_argument("The size of the input images are different");
    }

    // copy the input image
    edge_intensity = laplacian_img.clone();
    gradient_x = gradient_x.clone();
    gradient_y = gradient_y.clone();
}

// Today's goal
std::vector<EdgePoint> EdgeDetector::getKeyEdgePoints()
{
    std::vector<EdgePoint> key_edge_points;

    // divide the image into blocks
    int num_blocks_x = edge_intensity.cols / BLOCK_SIZE;
    int num_blocks_y = edge_intensity.rows / BLOCK_SIZE;

    for (int block_index_x = 0; block_index_x < num_blocks_x; block_index_x++)
    {
        for (int block_index_y = 0; block_index_y < num_blocks_y; block_index_y++)
        {
            // get edges in the block
            std::vector<Edge> edges = getEdges(block_index_x, block_index_y);

            // get key edge points
            for (const auto& edge : edges)
            {
                key_edge_points.push_back(edge.getMiddlePoint());
            }
        }
    }

    return key_edge_points;
}

std::vector<Edge> EdgeDetector::getEdges(const int block_index_x,
                                         const int block_index_y)
{
    // get the block
    cv::Rect block_rect(block_index_x * BLOCK_SIZE, block_index_y * BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE);
    cv::Mat block = edge_intensity(block_rect);

    // try to find the edge TRIAL_NUM times
    std::vector<Edge> edges;
    int failed_trial_num = 0;
    for (int i = 0; i < TRIAL_NUM; i++)
    {
        // find the maximum intensity point
        double min_val, max_val;
        cv::Point min_loc, max_loc;
        cv::minMaxLoc(block, &min_val, &max_val, &min_loc, &max_loc);

        // trace the edge
        Edge edge = traceEdge(cv::Point(max_loc.x + block_index_x * BLOCK_SIZE, max_loc.y + block_index_y * BLOCK_SIZE));

        // check edge size
        if (edge.size() < MIN_EDGE_SIZE)
        {
            failed_trial_num++;
            if (failed_trial_num > MAX_FAILED_TRIAL_NUM) break;
            continue;
        }

        // add the edge to the list
        edges.push_back(edge);

        // reduce the intensity of the area around the edge
        for (const auto& point : edge.getEdgePoints())
        {
            // transform to the block coordinate
            cv::Point block_point = point - cv::Point(block_index_x * BLOCK_SIZE, block_index_y * BLOCK_SIZE);

            cv::Rect intensity_rect(block_point.x - EDGE_CLEARANCE,
                                    block_point.y - EDGE_CLEARANCE,
                                    EDGE_CLEARANCE * 2 + 1,
                                    EDGE_CLEARANCE * 2 + 1);
            edge_intensity(intensity_rect) = 0;
        }
    }

    return edges;
}

Edge EdgeDetector::traceEdge(const cv::Point& start_point)
{
    // trace the edge
    Edge edge;
    float angle = getAngle(start_point);
    cv::Point current_point = start_point;
    edge.addPoint(current_point, angle);

    int max_num_points = edge_intensity.cols + edge_intensity.rows;

    // trace the edge in the forward direction
    for (int i = 0; i < max_num_points; i++)
    {
        cv::Point next_point;
        if (!getNextEdgePoint(current_point, angle, true, next_point))
        {
            break;
        }

        edge.points.push_back(next_point);
        current_point = next_point;
    }

    // trace the edge in the backward direction
    current_point = start_point;
    for (int i = 0; i < max_num_points; i++)
    {
        cv::Point next_point;
        if (!getNextEdgePoint(current_point, angle, false, next_point))
        {
            break;
        }

        edge.points.insert(edge.points.begin(), next_point);
        current_point = next_point;
    }

    return edge;
}

cv::Vec2f EdgeDetector::getGradient(const cv::Point& point)
{
    return cv::Vec2f(gradient_x.at<float>(point.y, point.x),
                     gradient_y.at<float>(point.y, point.x));
}

// return the next edge point
// current_point: the current point. The point should be enought far from the image boundary
// is_forward: true if the direction is forward, false if the direction is backward
bool getNextEdgePoint(const cv::Point& current_point,
                      const bool is_forward,
                      cv::Point& next_point)
{
    // return maximum intensity point in the 3 neighbors
    float max_intensity = 0;

    // edge direction is 90 deg rotated from gradient
    cv::Vec2f gradient = getGradient(current_point);
    cv::Vec2f edge_direction = cv::Vec2f(-gradient.y, gradient.x);

    std::vector<cv::Vec2f> directions = {cv::Vec2f(1, 0),
                                         cv::Vec2f(1, 1),
                                         cv::Vec2f(1, -1),
                                         cv::Vec2f(-1, 0),
                                         cv::Vec2f(-1, 1),
                                         cv::Vec2f(-1, -1),
                                         cv::Vec2f(0, 1),
                                         cv::Vec2f(0, -1)}

    for (const auto& direction : directions)
    {
        // continue if the direction is opposite to the edge direction
        if (direction.dot(edge_direction) < 0)
        {
            continue;
        }

        cv::Point neighbor_point = current_point + cv::Point(direction[0], direction[1]);
        float intensity = edge_intensity.at<float>(neighbor_point.y, neighbor_point.x);
        if (intensity > max_intensity)
        {
            max_intensity = intensity;
            next_point = neighbor_point;
        }
    }
    return true;
}
