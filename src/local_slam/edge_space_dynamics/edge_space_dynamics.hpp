#ifndef EDGE_SPACE_DYNAMICS_HPP
#define EDGE_SPACE_DYNAMICS_HPP

#include <Eigen/Dense>
#include <vector>
#include <algorithm> // std::max_element
#include <yaml-cpp/yaml.h>

#include "force_calculation.hpp"
#include "edge_node.hpp"
#include "pose_3d.hpp"
#include "line_3d.hpp"
#include "vector_average.hpp"

class EdgeSpaceDynamics {
public:
    EdgeSpaceDynamics(const std::string& config_file);

    bool get_frame_pose(std::vector<EdgeNode>& edge_nodes,
                        const float valid_edge_nodes_ratio_threshold,
                        Pose3D& frame_pose);

    bool add_new_edge(const Pose3D frame1_pose,
                      const Pose3D frame2_pose,
                      const EdgeNode frame1_edge_node,
                      const EdgeNode frame2_edge_node,
                      int& edge_id);

    bool optimize(Pose3D& frame_pose,
                  std::vector<EdgeNode>& edge_nodes,
                  const bool update_frame_pose);

    std::vector<Line3D> get_edge3ds();
    Line3D get_edge3d(int edge_id);
    int set_edge3d(Eigen::Vector3f start_point,
                   Eigen::Vector3f direction,
                   float length);

    bool calculate_frame_pose(std::vector<EdgeNode> edge_nodes,
                              Pose3D& frame_pose);

    void get_stress(std::vector<EdgeNode> edge_nodes,
                    Pose3D frame_pose,
                    std::vector<float>& translation_stress,
                    std::vector<float>& rotation_stress);

    void clear_edges();

private:
    std::vector<Line3D> edges;
    std::vector<int> edge_ids;

    void add_edge(const Line3D edge);

    bool get_force(Line3D edge,
                   EdgeNode edge_node,
                   Pose3D frame_pose,
                   Force3D& force_to_frame,
                   Force3D& force_to_edge,
                   float& torque_center_point_for_edge_line);

    void load_config(const std::string& config_file);

    // Configuration values
    int EDGE_NUM_TO_GET_FRAME_POSE;
    float TRANSLATION_STRESS_THRESHOLD_GAIN;
    float ROTATION_STRESS_THRESHOLD_GAIN;

    float INITIAL_EDGE_DISTANCE_FROM_FRAME;
    float EDGE_POSE_TRANSLATE_GAIN;
    float EDGE_POSE_ROTATE_GAIN;
    float EDGE_MIN_TRANSLATE_FORCE;
    float DELTA_EDGE_POSITION_TO_CHECK;
    float CAL_FINISH_TRANSLATE_VARIANCE;
    float CAL_FINISH_ROTATE_VARIANCE;

    int MAX_CAL_ITER;
    float FRAME_POSE_TRANSLATE_GAIN;
    float FRAME_POSE_ROTATE_GAIN;
    float FRAME_POSE_CAL_FINISH_TRANSLATE_VARIANCE;
    float FRAME_POSE_CAL_FINISH_TRANSLATIONAL_DELTA;
    float FRAME_POSE_CAL_FINISH_ROTATIONAL_DELTA;

    float CAL_FINISH_FORCE_SIZE;
    float CAL_FINISH_TORQUE_SIZE;
};

#endif // EDGE_SPACE_DYNAMICS_HPP

