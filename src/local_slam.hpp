#ifndef LOCAL_SLAM_HPP
#define LOCAL_SLAM_HPP

#define VALID_TRANSLATIONAL_DIFF 0.1f // for initialization
#define VALID_ROTATIONAL_DIFF 0.1f // for initialization

// for frame_node
#define WINDOW_SIZE 50
#define ANGLE_RESOLUTION 0.2f

// for edge_space_dynamics
#define VALID_EDGE_NODES_RATIO_THRESHOLD 0.6f

#include <opencv2/opencv.hpp>

#include "frame_node.hpp"
#include "frame_node_data.hpp"
#include "edge_space_dynamics.hpp"
#include "camera_model.hpp"
#include "vslam_debug_view.hpp"

class LocalSlam {
public:
    LocalSlam(const CameraModel& camera_model,
              const std::string& edge_space_dynamics_config_file);

    bool multi_frame_init(const cv::Mat& image);

    // 1. make Frame instance
    // 2. find matching edges to the last key frame
    // 3. calculate the pose of the current frame
    // 4. check if the current frame is a key frame. if not, return the pose of the current frame
    // 5. calculate first matched edges pose and register them to edge space dynamics.
    // 6. return the pose of the current frame
    bool update(const cv::Mat& image,
                const Pose3D& external_pose_data,
                const bool use_external_pose_data,
                const bool calculate_pose,
                Pose3D& pose);

    // TODO: add update with external pose data
    // bool update(const cv::Mat& image,
    //             Pose3D& pose,
    //             const Pose3D& external_pose_data);

    void optimize(const int iteration);

    void fix_edges(FrameNode& frame_node1,
                   FrameNode& frame_node2,
                   const Pose3D& pose_frame1,
                   const Pose3D& pose_frame2);

    std::vector<Line3D> get_fixed_edges();

    void save_log(const std::string& path_to_dir);

    VslamDebugView get_current_debug_view(std::string& file_name);
    VslamDebugView get_key_frame_debug_view(std::string& file_name);
    VslamDebugView get_third_person_view(const Pose3D& camera_pose,
                                         cv::Mat& base_image,
                                         cv::Mat& camera_matrix,
                                         std::string& file_name);

private:
    int frame_count = 0;

    bool did_finish_initilization = false;

    EdgeSpaceDynamics edge_space_dynamics;

    CameraModel camera_model;

    std::vector<FrameNodeData> key_frames;

    void update_latest_n_frame_nodes(const FrameNode& frame_node);

    bool get_pose(const FrameNode& frame_node,
                  Pose3D& pose);

    bool calculate_first_matched_edges(const FrameNode& last_key_frame,
                                 const FrameNode& current_frame);

    void add_key_frame(FrameNodeData);

    FrameNode current_frame_node; // for debug view

    std::vector<std::pair<EdgePoint, EdgePoint>> rejected_edge_points; // for debug view

    std::vector<EdgePoint> unmatched_edge_points; // for debug view

    std::vector<EdgePoint> found_edge_points; // for debug view
};

#endif // LOCAL_SLAM_HPP
