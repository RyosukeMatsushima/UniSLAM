#include "edge_space_dynamics.hpp"

#include <iostream>

EdgeSpaceDynamics::EdgeSpaceDynamics(const std::string& config_file) {
    load_config(config_file);
}

void EdgeSpaceDynamics::load_config(const std::string& config_file) {
    YAML::Node config = YAML::LoadFile(config_file);

    try {
        EDGE_NUM_TO_GET_FRAME_POSE = config["EDGE_NUM_TO_GET_FRAME_POSE"].as<int>();
        TRANSLATION_STRESS_THRESHOLD_GAIN = config["TRANSLATION_STRESS_THRESHOLD_GAIN"].as<float>();
        ROTATION_STRESS_THRESHOLD_GAIN = config["ROTATION_STRESS_THRESHOLD_GAIN"].as<float>();

        INITIAL_EDGE_DISTANCE_FROM_FRAME = config["INITIAL_EDGE_DISTANCE_FROM_FRAME"].as<float>();
        EDGE_POSE_TRANSLATE_GAIN = config["EDGE_POSE_TRANSLATE_GAIN"].as<float>();
        EDGE_POSE_ROTATE_GAIN = config["EDGE_POSE_ROTATE_GAIN"].as<float>();
        EDGE_MIN_TRANSLATE_FORCE = config["EDGE_MIN_TRANSLATE_FORCE"].as<float>();
        DELTA_EDGE_POSITION_TO_CHECK = config["DELTA_EDGE_POSITION_TO_CHECK"].as<float>();
        CAL_FINISH_TRANSLATE_VARIANCE = config["CAL_FINISH_TRANSLATE_VARIANCE"].as<float>();
        CAL_FINISH_ROTATE_VARIANCE = config["CAL_FINISH_ROTATE_VARIANCE"].as<float>();

        MAX_CAL_ITER = config["MAX_CAL_ITER"].as<int>();
        FRAME_POSE_TRANSLATE_GAIN = config["FRAME_POSE_TRANSLATE_GAIN"].as<float>();
        FRAME_POSE_ROTATE_GAIN = config["FRAME_POSE_ROTATE_GAIN"].as<float>();
        CAL_FINISH_FORCE_SIZE = config["CAL_FINISH_FORCE_SIZE"].as<float>();
        CAL_FINISH_TORQUE_SIZE = config["CAL_FINISH_TORQUE_SIZE"].as<float>();
    } catch (YAML::Exception& e) {
        std::cerr << "Error in loading config file: " << e.what() << std::endl;
        std::cerr << "You may miss some parameters in the config yaml file." << std::endl;
    }
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
    // calculate the closest point between the two lines as the initial position of the edge
    Line3D detect_line_from_frame1(0,
                                   frame1_pose.position,
                                   frame1_pose.rotateVectorToWorld(frame1_edge_node.direction_frame_to_edge),
                                   0);
    Line3D detect_line_from_frame2(1,
                                   frame2_pose.position,
                                   frame2_pose.rotateVectorToWorld(frame2_edge_node.direction_frame_to_edge),
                                   0);
    float closest_point1, closest_point2;
    if (!Line3D::get_closest_points_between(detect_line_from_frame1,
                                            detect_line_from_frame2,
                                            closest_point1,
                                            closest_point2)) return false;

    Line3D edge(edges.size(),
                detect_line_from_frame1.get_point_at(closest_point1),
                frame1_pose.rotateVectorToWorld(Eigen::Vector3f(frame1_edge_node.edge_direction[0], frame1_edge_node.edge_direction[1], 0)),
                0);

    bool cal_finish = false;

    VectorAverage translation_force_average(1); // TODO: set parameter
    VectorAverage edge_start_point_average(10); // TODO: set parameter
    VectorAverage edge_direction_average(10); // TODO: set parameter
    int cal_finish_count = 0;
    for (int i = 0; i < MAX_CAL_ITER; i++) {

        Line3D current_edge = edge.clone();

        // TODO: refactor this part. double same code
        Force3D force_to_frame_not_used;
        Force3D force_to_edge_with_frame1;
        float torque_center_point_for_edge_line_with_frame1;

        if (!get_force(current_edge,
                       frame1_edge_node,
                       frame1_pose,
                       force_to_frame_not_used,
                       force_to_edge_with_frame1,
                       torque_center_point_for_edge_line_with_frame1)) return false;

        force_to_edge_with_frame1.force = force_to_edge_with_frame1.force * EDGE_POSE_TRANSLATE_GAIN;
        force_to_edge_with_frame1.torque = force_to_edge_with_frame1.torque * EDGE_POSE_ROTATE_GAIN;

        // TODO: remove this line
        edge.add_force(Eigen::Vector3f(0, 0, 0),
                       force_to_edge_with_frame1.torque,
                       torque_center_point_for_edge_line_with_frame1);

        Force3D force_to_edge_with_frame2;
        float torque_center_point_for_edge_line_with_frame2;

        if (!get_force(current_edge,
                       frame2_edge_node,
                       frame2_pose,
                       force_to_frame_not_used,
                       force_to_edge_with_frame2,
                       torque_center_point_for_edge_line_with_frame2)) return false;

        force_to_edge_with_frame2.force = force_to_edge_with_frame2.force * EDGE_POSE_TRANSLATE_GAIN;
        force_to_edge_with_frame2.torque = force_to_edge_with_frame2.torque * EDGE_POSE_ROTATE_GAIN;

        // TODO: remove this line
        edge.add_force(Eigen::Vector3f(0, 0, 0),
                       force_to_edge_with_frame2.torque,
                       torque_center_point_for_edge_line_with_frame2);


        // TODO: add force_sum to the edge.
        Force3D force_sum;
        force_sum.add(force_to_edge_with_frame1);
        force_sum.add(force_to_edge_with_frame2);

        translation_force_average.add_vector(force_sum.force);

        Eigen::Vector3f actual_force = translation_force_average.get_average();
        if (actual_force.norm() < EDGE_MIN_TRANSLATE_FORCE) {
            actual_force = actual_force.normalized() * EDGE_MIN_TRANSLATE_FORCE;
        }

        edge.add_force(actual_force,
                       Eigen::Vector3f(0, 0, 0),
                       0.0f);

        edge_start_point_average.add_vector(edge.start_point());
        edge_direction_average.add_vector(edge.direction());

        if (!edge_start_point_average.is_filled()) continue;
        if (!edge_direction_average.is_filled()) continue;

        if (edge_start_point_average.get_variance().norm() < CAL_FINISH_TRANSLATE_VARIANCE &&
            edge_direction_average.get_variance().norm() < CAL_FINISH_ROTATE_VARIANCE) {
            cal_finish = true;
            break;
        }
    }

    if (!cal_finish) return false;

    // check the edge is really fixed
    // the edge is fixed if the force will increase when the edge is moved to direction_frame_to_edge

    Line3D edge_moved = edge.clone();

    edge_moved.move(frame1_pose.rotateVectorToWorld(frame1_edge_node.direction_frame_to_edge * DELTA_EDGE_POSITION_TO_CHECK));

    Force3D force_to_edge_moved;
    Force3D force_to_frame_not_used;
    float torque_center_point_for_edge_line_moved;

    if (!get_force(edge_moved,
                   frame2_edge_node,
                   frame2_pose,
                   force_to_frame_not_used,
                   force_to_edge_moved,
                   torque_center_point_for_edge_line_moved)) return false;

    if ((force_to_edge_moved.force * EDGE_POSE_TRANSLATE_GAIN).cwiseAbs2().sum() < CAL_FINISH_FORCE_SIZE &&
        (force_to_edge_moved.torque * EDGE_POSE_ROTATE_GAIN).cwiseAbs2().sum() < CAL_FINISH_TORQUE_SIZE) {
        return false;
    }

    edges.push_back(edge);
    edge_ids.push_back(edge.id());
    edge_id = edge.id();
    return true;
}

bool EdgeSpaceDynamics::optimize(Pose3D& frame_pose,
                                 std::vector<EdgeNode>& edge_nodes,
                                 const bool update_frame_pose) {

    Pose3D current_frame_pose = frame_pose.clone();

    Force3D force_to_frame_sum, force_to_edge_sum;

    for (int i = 0; i < edge_nodes.size(); i++) {
        EdgeNode edge_node = edge_nodes[i];
        Line3D edge = edges[edge_node.edge_id];

        Force3D force_to_frame;
        Force3D force_to_edge;
        float torque_center_point_for_edge_line;

        if (!get_force(edge,
                       edge_node,
                       current_frame_pose,
                       force_to_frame,
                       force_to_edge,
                       torque_center_point_for_edge_line)) return false;

        force_to_frame_sum.add(force_to_frame);
        force_to_edge_sum.add(force_to_edge);
        edges[edge_node.edge_id].add_force(force_to_edge.force * EDGE_POSE_TRANSLATE_GAIN,
                                           force_to_edge.torque * EDGE_POSE_ROTATE_GAIN,
                                           torque_center_point_for_edge_line);
    }

    if (update_frame_pose) {
        frame_pose.translate(force_to_frame_sum.force * FRAME_POSE_TRANSLATE_GAIN / edge_nodes.size());
        frame_pose.rotate(force_to_frame_sum.torque * FRAME_POSE_ROTATE_GAIN / edge_nodes.size());
    }

    return true;
}

std::vector<Line3D> EdgeSpaceDynamics::get_edge3ds() {
    return edges;
}

Line3D EdgeSpaceDynamics::get_edge3d(int edge_id) {
    return edges[edge_id];
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

        // TODO: add cal_finish condition
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

void EdgeSpaceDynamics::clear_edges() {
    edges.clear();
    edge_ids.clear();
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

