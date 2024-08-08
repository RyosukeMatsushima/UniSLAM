#include "camera_model.hpp"


CameraModel::CameraModel(const cv::Mat& camera_matrix,
                         const cv::Size& img_size) :
    camera_matrix(camera_matrix),
    img_size(img_size) {
}

EdgeNode CameraModel::getEdgeNode(const EdgePoint& edge_point) const {
    Eigen::Vector3f direction_frame_to_edge((edge_point.point.x - float(camera_matrix.at<double>(0, 2))) / float(camera_matrix.at<double>(0, 0)),
                                            (edge_point.point.y - float(camera_matrix.at<double>(1, 2))) / float(camera_matrix.at<double>(1, 1)),
                                            1.0f);
    direction_frame_to_edge.normalize();

    Eigen::Vector2f edge_direction(edge_point.direction[0], edge_point.direction[1]);

    return EdgeNode(direction_frame_to_edge, edge_direction, edge_point.id);
}




