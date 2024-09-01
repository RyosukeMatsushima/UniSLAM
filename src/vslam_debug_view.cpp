
#include "vslam_debug_view.hpp"

VslamDebugView::VslamDebugView(const cv::Mat& base_img) : DebugView(base_img) {}

void Pose3D2cv(const Pose3D& pose, cv::Mat& rvec, cv::Mat& tvec) {
    Eigen::Matrix3f rotation_matrix = pose.orientation.toRotationMatrix();
    rvec = (cv::Mat_<double>(3, 3) << rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
                                      rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
                                      rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));
    tvec = (cv::Mat_<double>(3, 1) << pose.position.x(), pose.position.y(), pose.position.z());
}

void VslamDebugView::drawEdge3D(const Line3D& edge_3d, const Pose3D& pose, const cv::InputArray& camera_matrix, const cv::Scalar& color) {

    Eigen::Vector3f start_point = edge_3d.start_point();
    float min_length = 0.3f;
    Eigen::Vector3f end_point = edge_3d.get_point_at(std::max(min_length, edge_3d.length()));

    // calculate projected points
    cv::Mat rvec;
    cv::Mat tvec;

    Pose3D2cv(pose, rvec, tvec);

    // invert rotation matrix
    rvec = rvec.inv();
    tvec = -tvec;

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

struct PoseVisual {
    cv::Point3f origin;

    cv::Point3f x_axis;
    cv::Point3f y_axis;
    cv::Point3f z_axis;

    PoseVisual(const Pose3D& pose, const float size) {
        origin = cv::Point3f(-pose.position.x(), -pose.position.y(), -pose.position.z());

        Eigen::Matrix3f rotation_matrix = pose.orientation.toRotationMatrix().inverse();

        // size
        x_axis = cv::Point3f(rotation_matrix(0, 0), rotation_matrix(1, 0), rotation_matrix(2, 0)) * size + origin;
        y_axis = cv::Point3f(rotation_matrix(0, 1), rotation_matrix(1, 1), rotation_matrix(2, 1)) * size + origin;
        z_axis = cv::Point3f(rotation_matrix(0, 2), rotation_matrix(1, 2), rotation_matrix(2, 2)) * size + origin;
    }
};

void VslamDebugView::drawPose3D(const Pose3D& pose, const Pose3D& camera_pose, const cv::InputArray& camera_matrix, const cv::Scalar& color, const float size) {
    PoseVisual pose_visual(pose, size);

    cv::Mat rvec;
    cv::Mat tvec;

    Pose3D2cv(camera_pose, rvec, tvec);

    rvec = rvec.inv();
    tvec = -tvec;

    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F); // Assuming no lens distortion for now

    std::vector<cv::Point3f> points;

    points.push_back(pose_visual.origin);
    points.push_back(pose_visual.x_axis);
    points.push_back(pose_visual.y_axis);
    points.push_back(pose_visual.z_axis);

    std::vector<cv::Point2f> projected_points;

    cv::projectPoints(points, rvec, tvec, camera_matrix, dist_coeffs, projected_points);

    cv::line(base_img_, projected_points[0], projected_points[1], cv::Scalar(0, 0, 255), 2);
    cv::line(base_img_, projected_points[0], projected_points[2], cv::Scalar(0, 255, 0), 2);
    cv::line(base_img_, projected_points[0], projected_points[3], cv::Scalar(255, 0, 0), 2);

    // draw dot at origin
    cv::circle(base_img_, projected_points[0], 5, color, -1);
}

void VslamDebugView::addDiscriptionText(const std::string& text) {
    cv::putText(base_img_, text, cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 1);
}


