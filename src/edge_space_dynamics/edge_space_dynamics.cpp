#include "edge_space_dynamics.hpp"


EdgeSpaceDynamics::EdgeSpaceDynamics() {
}

// it's better if edge_nodes is suffuled before calling this function
bool EdgeSpaceDynamics::get_frame_pose(std::vector<EdgeNode>& edge_nodes,
                                       const float valid_edge_nodes_ratio_threshold,
                                       Pose3D& frame_pose) {

    if (edge_nodes.size() < EDGE_NUM_TO_GET_FRAME_POSE) {
        throw std::invalid_argument("edge_nodes.size() < EDGE_NUM_TO_GET_FRAME_POSE");
    }

    bool frame_pose_is_valid = false;

    float min_translation_stress = std::numeric_limits<float>::max();
    float min_rotation_stress = std::numeric_limits<float>::max();

    for (int edge_pointer = 0; edge_pointer < edge_nodes.size() - EDGE_NUM_TO_GET_FRAME_POSE; edge_pointer++) {

        // get EDGE_NUM_TO_GET_FRAME_POSE edges to calculate frame_pose
        std::vector<EdgeNode> edge_nodes_to_calculate;
        for (int i = 0; i < EDGE_NUM_TO_GET_FRAME_POSE; i++) {
            edge_nodes_to_calculate.push_back(edge_nodes[edge_pointer + i]);
        }
        Pose3D current_frame_pose = frame_pose.clone();
        if (!calculate_frame_pose(edge_nodes_to_calculate, current_frame_pose)) continue;


        // check if the calculated frame_pose is valid
        std::vector<float> translation_stress_with_calculated_edges, rotation_stress_with_calculated_edges;
        get_stress(edge_nodes_to_calculate,
                   current_frame_pose,
                   translation_stress_with_calculated_edges,
                   rotation_stress_with_calculated_edges);

        float current_max_translation_stress = *std::max_element(translation_stress_with_calculated_edges.begin(),
                                                                 translation_stress_with_calculated_edges.end());
        float current_max_rotation_stress = *std::max_element(rotation_stress_with_calculated_edges.begin(),
                                                              rotation_stress_with_calculated_edges.end());

        if (current_max_translation_stress > min_translation_stress ||
            current_max_rotation_stress > min_rotation_stress) {
            continue;
        }


        float translation_stress_threshold = current_max_translation_stress * TRANSLATION_STRESS_THRESHOLD_GAIN;
        float rotation_stress_threshold = current_max_rotation_stress * ROTATION_STRESS_THRESHOLD_GAIN;

        // calculate the stress with the all of the edges
        std::vector<float> translation_stress_with_all_edges, rotation_stress_with_all_edges;
        get_stress(edge_nodes,
                   current_frame_pose,
                   translation_stress_with_all_edges,
                   rotation_stress_with_all_edges);

        // it's valid edge_nodes if the stress is less than the threshold
        int valid_edge_nodes_count = 0;
        for (int i = 0; i < edge_nodes.size(); i++) {
            if (translation_stress_with_all_edges[i] <= translation_stress_threshold &&
                rotation_stress_with_all_edges[i] <= rotation_stress_threshold) {
                edge_nodes[i].is_valid = true;
                valid_edge_nodes_count++;
            } else {
                edge_nodes[i].is_valid = false;
            }
        }

        float valid_edge_nodes_ratio = (float)valid_edge_nodes_count / float(edge_nodes.size());

        if (valid_edge_nodes_ratio > valid_edge_nodes_ratio_threshold) {
            min_translation_stress = current_max_translation_stress;
            min_rotation_stress = current_max_rotation_stress;
            current_frame_pose.copy_to(frame_pose);
            frame_pose_is_valid = true;
        }
    }

    return frame_pose_is_valid;
}

bool EdgeSpaceDynamics::add_new_edge(Pose3D frame1_pose,
                                     Pose3D frame2_pose,
                                     EdgeNode frame1_edge_node,
                                     EdgeNode frame2_edge_node,
                                     int& edge_id) {
    return true;
}

Pose3D EdgeSpaceDynamics::optimize(Pose3D frame_pose,
                                   std::vector<EdgeNode> edge_nodes) {
    return frame_pose;
}

void EdgeSpaceDynamics::get_edge3ds(std::vector<Line3D>& edge3ds) {
}

int EdgeSpaceDynamics::set_edge3d(Eigen::Vector3f start_point,
                                  Eigen::Vector3f direction,
                                  float length) {
    int edge_id = edges.size();
    Line3D edge3d(edge_id, start_point, direction, length);
    edges.push_back(edge3d);
    edge_ids.push_back(edge_id);
    return edge_id;
}

bool EdgeSpaceDynamics::calculate_frame_pose(std::vector<EdgeNode> edge_nodes,
                                             Pose3D& frame_pose) {
    for (int i = 0; i < MAX_CAL_ITER; i++) {
        Pose3D current_frame_pose = frame_pose.clone();

        Force3D force_to_frame_sum, force_to_edge_sum;

        for (int j = 0; j < edge_nodes.size(); j++) {
            EdgeNode edge_node = edge_nodes[j];
            Line3D edge = edges[edge_node.edge_id];

            Force3D force_to_frame;
            Force3D force_to_edge;
            float torque_center_point_for_edge_line;

            if (!get_force(edge,
                           edge_node,
                           frame_pose,
                           force_to_frame,
                           force_to_edge,
                           torque_center_point_for_edge_line)) return false;

            force_to_frame_sum.add(force_to_frame);
            force_to_edge_sum.add(force_to_edge);
        }

        frame_pose.translate(force_to_frame_sum.force * FRAME_POSE_TRANSLATE_GAIN / edge_nodes.size());
        frame_pose.rotate(force_to_frame_sum.torque * FRAME_POSE_ROTATE_GAIN / edge_nodes.size());
    }
    return true;
}

void EdgeSpaceDynamics::get_stress(std::vector<EdgeNode> edge_nodes,
                                   Pose3D frame_pose,
                                   std::vector<float>& translation_stress,
                                   std::vector<float>& rotation_stress) {
    for (auto edge_node : edge_nodes) {
        Line3D edge = edges[edge_node.edge_id];

        Force3D force_to_frame;
        Force3D force_to_edge;
        float torque_center_point_for_edge_line;

        if (!get_force(edge,
                       edge_node,
                       frame_pose,
                       force_to_frame,
                       force_to_edge,
                       torque_center_point_for_edge_line)) {
            translation_stress.push_back(0);
            rotation_stress.push_back(0);
            continue;
        }

        float translation_stress_value = force_to_frame.force.norm();
        float rotation_stress_value = force_to_frame.torque.norm();

        translation_stress.push_back(translation_stress_value);
        rotation_stress.push_back(rotation_stress_value);
    }
}

bool EdgeSpaceDynamics::get_force(Line3D edge,
                                  EdgeNode edge_node,
                                  Pose3D frame_pose,
                                  Force3D& force_to_frame,
                                  Force3D& force_to_edge,
                                  float& torque_center_point_for_edge_line) {

    ForceCalculation force_calculation(edge,
                                       edge_node,
                                       frame_pose);
    if (!force_calculation.calculate()) return false;

    force_to_frame = force_calculation.getForceToFrame();
    force_to_edge = force_calculation.getForceToEdge();
    torque_center_point_for_edge_line = force_calculation.getTorqueCenterPointForEdgeLine();
    return true;
}

