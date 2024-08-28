
#include "vslam_debug_view.hpp"

VslamDebugView::VslamDebugView(const cv::Mat& base_img) : DebugView(base_img) {}

void VslamDebugView::drawEdge3D(const Line3D& edge_3d, const Pose3D& pose, const cv::InputArray& camera_matrix, const cv::Scalar& color) {

    Eigen::Vector3f start_point = edge_3d.start_point();
    float min_length = 0.1f;
    Eigen::Vector3f end_point = edge_3d.get_point_at(std::max(min_length, edge_3d.length()));

    // calculate projected points
    Eigen::Matrix3f rotation_matrix = pose.orientation.toRotationMatrix();
    cv::Mat rvec = (cv::Mat_<double>(3, 3) << rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
                                              rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
                                              rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));
    // invert rotation matrix
    rvec = rvec.inv();

    cv::Mat tvec = (cv::Mat_<double>(3, 1) << -pose.position.x(), -pose.position.y(), -pose.position.z());

    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F); // Assuming no lens distortion for now

    std::vector<cv::Point3f> points;

    points.push_back(cv::Point3f(start_point.x(), start_point.y(), start_point.z()));
    points.push_back(cv::Point3f(end_point.x(), end_point.y(), end_point.z()));

    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(points, rvec, tvec, camera_matrix, dist_coeffs, projected_points);

    // draw arrow
    cv::arrowedLine(base_img_, projected_points[0], projected_points[1], color, 2);

    // draw id
    cv::putText(base_img_, std::to_string(edge_3d.id()), projected_points[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
}
