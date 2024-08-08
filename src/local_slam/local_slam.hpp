#ifndef LOCAL_SLAM_HPP
#define LOCAL_SLAM_HPP

#define VALID_TRANSLATIONAL_DIFF 0.1f // for initialization
#define VALID_ROTATIONAL_DIFF 0.1f // for initialization

// for frame_node
#define WINDOW_SIZE 200
#define ANGLE_RESOLUTION 0.2f

// for edge_space_dynamics
#define VALID_EDGE_NODES_RATIO_THRESHOLD 0.6f

#include <opencv2/opencv.hpp>

#include "frame_node.hpp"
#include "edge_space_dynamics.hpp"
#include "camera_model.hpp"
#include "debug_view.hpp"

class LocalSlam {
public:
    LocalSlam(const CameraModel& camera_model,
              const std::string& edge_space_dynamics_config_file);

    bool multi_frame_init(const cv::Mat& image,
                          const Eigen::Vector3f& external_position_data,
                          const Eigen::Quaternionf& external_orientation_data);

    // 1. make Frame instance
    // 2. find matching edges to the last key frame
    // 3. calculate the pose of the current frame
    // 4. check if the current frame is a key frame. if not, return the pose of the current frame
    // 5. calculate first matched edges pose and register them to edge space dynamics.
    // 6. return the pose of the current frame
    bool update(const cv::Mat& image,
                Pose3D& pose);

    // TODO: add update with external pose data
    // bool update(const cv::Mat& image,
    //             Pose3D& pose,
    //             const Pose3D& external_pose_data);

    void optimize();

    void fix_edges(FrameNode& frame_node1,
                   FrameNode& frame_node2,
                   const Pose3D& pose_frame1,
                   const Pose3D& pose_frame2);

    std::vector<Line3D> get_fixed_edges();

    void save_log(const std::string& path_to_dir);

private:
    int frame_count = 0;

    EdgeSpaceDynamics edge_space_dynamics;

    CameraModel camera_model;

    std::vector<std::pair<FrameNode, Pose3D>> key_frames;

    FrameNode current_frame_node;

    void update_latest_n_frame_nodes(const FrameNode& frame_node);

    bool get_pose(const FrameNode& frame_node,
                  Pose3D& pose);

    bool calculate_first_matched_edges(const FrameNode& last_key_frame,
                                 const FrameNode& current_frame);
};

#endif // LOCAL_SLAM_HPP
