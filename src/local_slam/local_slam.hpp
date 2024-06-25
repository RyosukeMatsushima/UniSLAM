#ifndef LOCAL_SLAM_HPP
#define LOCAL_SLAM_HPP

#include <opencv2/opencv.hpp>

#include "frame_node.hpp"
#include "edge_space_dynamics.hpp"

class LocalSlam {
public:
    LocalSlam();

    bool multi_frame_init(const cv::Mat& image);

    // 1. make Frame instance
    // 2. find matching edges to the last key frame
    // 3. calculate the pose of the current frame
    // 4. check if the current frame is a key frame. if not, return the pose of the current frame
    // 5. calculate first matched edges pose and register them to edge space dynamics.
    // 6. return the pose of the current frame
    bool update(const cv::Mat& image,
                Pose3D& pose);
    void optimize();

private:
    EdgeSpaceDynamics edge_space_dynamics;

//TODO: add CameraModel    CameraModel camera_model;

    std::vector<FrameNode> key_frames;

    bool get_pose(const FrameNode& frame_node,
                  Pose3D& pose);

    bool calculate_first_matched_edges(const FrameNode& last_key_frame,
                                 const FrameNode& current_frame);
};

#endif // LOCAL_SLAM_HPP
