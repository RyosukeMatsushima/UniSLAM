#ifndef VSLAM_DEBUG_VIEW_HPP
#define VSLAM_DEBUG_VIEW_HPP

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "debug_view.hpp"
#include "line_3d.hpp"
#include "pose_3d.hpp"


class VslamDebugView : public DebugView {
public:
    VslamDebugView(const cv::Mat& base_img);

    void drawEdge3D(const Line3D& edge_3d, const Pose3D& pose, const cv::InputArray& camera_matrix, const cv::Scalar& color);

    void drawPose3D(const Pose3D& pose, const Pose3D& camera_pose, const cv::InputArray& camera_matrix, const cv::Scalar& color, const float size);

    void addDiscriptionText(const std::string& text);
};

#endif // VSLAM_DEBUG_VIEW_HPP
