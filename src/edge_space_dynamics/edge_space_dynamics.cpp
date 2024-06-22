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
        std::vector<bool> is_valid_edge_nodes(edge_nodes.size());

        for (int i = 0; i < edge_nodes.size(); i++) {
            is_valid_edge_nodes[i] = translation_stress_with_all_edges[i] <= translation_stress_threshold &&
                                     rotation_stress_with_all_edges[i] <= rotation_stress_threshold;
        }

        int valid_edge_nodes_count = std::count(is_valid_edge_nodes.begin(), is_valid_edge_nodes.end(), true);

        float valid_edge_nodes_ratio = (float)valid_edge_nodes_count / float(edge_nodes.size());

        // update the frame_pose if the valid_edge_nodes_ratio is higher than the threshold
        if (valid_edge_nodes_ratio < valid_edge_nodes_ratio_threshold) continue;

        for (int i = 0; i < edge_nodes.size(); i++) {
            edge_nodes[i].is_valid = is_valid_edge_nodes[i];
        }

        min_translation_stress = current_max_translation_stress;
        min_rotation_stress = current_max_rotation_stress;
        current_frame_pose.copy_to(frame_pose);
        frame_pose_is_valid = true;
    }

    return frame_pose_is_valid;
}

bool EdgeSpaceDynamics::add_new_edge(const Pose3D frame1_pose,
                                     const Pose3D frame2_pose,
                                     const EdgeNode frame1_edge_node,
                                     const EdgeNode frame2_edge_node,
                                     int& edge_id) {

    Line3D edge(edges.size(),
                frame1_pose.transformToWorld(frame1_edge_node.direction_frame_to_edge * INITIAL_EDGE_DISTANCE_FROM_FRAME1),
                frame1_pose.rotateVectorToWorld(Eigen::Vector3f(frame1_edge_node.edge_direction[0], frame1_edge_node.edge_direction[1], 0)),
                0);

    bool cal_finish = false;
    for (int i = 0; i < MAX_CAL_ITER; i++) {

        Force3D force_to_frame_not_used;
        Force3D force_to_edge_with_frame1;
        float torque_center_point_for_edge_line_with_frame1;

        if (!get_force(edge,
                       frame1_edge_node,
                       frame1_pose,
                       force_to_frame_not_used,
                       force_to_edge_with_frame1,
                       torque_center_point_for_edge_line_with_frame1)) return false;

        edge.add_force(force_to_edge_with_frame1.force * EDGE_POSE_TRANSLATE_GAIN,
                       force_to_edge_with_frame1.torque * EDGE_POSE_ROTATE_GAIN,
                       torque_center_point_for_edge_line_with_frame1);

        Force3D force_to_edge_with_frame2;
        float torque_center_point_for_edge_line_with_frame2;

        if (!get_force(edge,
                       frame2_edge_node,
                       frame2_pose,
                       force_to_frame_not_used,
                       force_to_edge_with_frame2,
                       torque_center_point_for_edge_line_with_frame2)) return false;

        edge.add_force(force_to_edge_with_frame2.force * EDGE_POSE_TRANSLATE_GAIN,
                       force_to_edge_with_frame2.torque * EDGE_POSE_ROTATE_GAIN,
                       torque_center_point_for_edge_line_with_frame2);

        if (force_to_edge_with_frame1.force.norm() < CAL_FINISH_FORCE_SIZE &&
            force_to_edge_with_frame1.torque.norm() < CAL_FINISH_TORQUE_SIZE &&
            force_to_edge_with_frame2.force.norm() < CAL_FINISH_FORCE_SIZE &&
            force_to_edge_with_frame2.torque.norm() < CAL_FINISH_TORQUE_SIZE) {
            cal_finish = true;
            break;
        }

    }

    edges.push_back(edge);
    edge_ids.push_back(edge.id());
    edge_id = edge.id();
    return cal_finish;
}

bool EdgeSpaceDynamics::optimize(Pose3D& frame_pose,
                                 std::vector<EdgeNode>& edge_nodes,
                                 const bool update_frame_pose) {

    return true;
}

std::vector<Line3D> EdgeSpaceDynamics::get_edge3ds() {
    return edges;
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

        // TODO: remove force_to_edge_sum, it's not used
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

