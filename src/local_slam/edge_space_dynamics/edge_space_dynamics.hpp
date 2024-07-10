#ifndef EDGE_SPACE_DYNAMICS_HPP
#define EDGE_SPACE_DYNAMICS_HPP

// for get_frame_pose
#define EDGE_NUM_TO_GET_FRAME_POSE 4
#define TRANSLATION_STRESS_THRESHOLD_GAIN 1.1f
#define ROTATION_STRESS_THRESHOLD_GAIN 1.1f

// for add_new_edge
#define INITIAL_EDGE_DISTANCE_FROM_FRAME1 0.1f
#define EDGE_POSE_TRANSLATE_GAIN 0.1f
#define EDGE_POSE_ROTATE_GAIN 0.1f
#define DELTA_EDGE_POSITION_TO_CHECK 0.1f

// for calculate_frame_pose
#define MAX_CAL_ITER 1000
#define FRAME_POSE_TRANSLATE_GAIN 1.8f
#define FRAME_POSE_ROTATE_GAIN 0.1f

// TODO: use this value to stop the calculation
#define CAL_FINISH_FORCE_SIZE 0.001
#define CAL_FINISH_TORQUE_SIZE 0.001

#include <Eigen/Dense>
#include <vector>
#include <algorithm> // std::max_element

#include "force_calculation.hpp"
#include "edge_node.hpp"
#include "pose_3d.hpp"
#include "line_3d.hpp"


class EdgeSpaceDynamics {
public:

    EdgeSpaceDynamics();

    // usecase0
    // Initialize with first 2 frames
    // Returns the calculated edges pointer
    // bool double_frame_init(); // TODO: this will be needed with the devicewhich does not have imu or external pose sensor

    // usecase1
    // Calculate the pose of the new frame
    // The new frame should be related to the calculated edges
    bool get_frame_pose(std::vector<EdgeNode>& edge_nodes,
                        const float valid_edge_nodes_ratio_threshold,
                        Pose3D& frame_pose);

    // usecase2
    // Add new edge
    // returns the edge pointer
    // returns -1 if the edge is not added
    bool add_new_edge(const Pose3D frame1_pose,
                      const Pose3D frame2_pose,
                      const EdgeNode frame1_edge_node,
                      const EdgeNode frame2_edge_node,
                      int& edge_id);

    // usecase5
    // optimize the frame pose and edge pose
    bool optimize(Pose3D& frame_pose,
                  std::vector<EdgeNode>& edge_nodes,
                  const bool update_frame_pose);

    // usecase3
    std::vector<Line3D> get_edge3ds();

    Line3D get_edge3d(int edge_id);

    // usecase4
    // returns id of the edge
    int set_edge3d(Eigen::Vector3f start_point,
                   Eigen::Vector3f direction,
                   float length);

    bool calculate_frame_pose(std::vector<EdgeNode> edge_nodes,
                              Pose3D& frame_pose);

    void get_stress(std::vector<EdgeNode> edge_nodes,
                    Pose3D frame_pose,
                    std::vector<float>& translation_stress,
                    std::vector<float>& rotation_stress);

private:

    std::vector<Line3D> edges;
    std::vector<int> edge_ids;

    bool get_force(Line3D edge,
                   EdgeNode edge_node,
                   Pose3D frame_pose,
                   Force3D& force_to_frame,
                   Force3D& force_to_edge,
                   float& torque_center_point_for_edge_line);

};

#endif // EDGE_SPACE_DYNAMICS_HPP

