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
        FRAME_POSE_CAL_FINISH_TRANSLATE_VARIANCE = config["FRAME_POSE_CAL_FINISH_TRANSLATE_VARIANCE"].as<float>();
        FRAME_POSE_CAL_FINISH_TRANSLATIONAL_DELTA = config["FRAME_POSE_CAL_FINISH_TRANSLATIONAL_DELTA"].as<float>();
        FRAME_POSE_CAL_FINISH_ROTATIONAL_DELTA = config["FRAME_POSE_CAL_FINISH_ROTATIONAL_DELTA"].as<float>();

        EXTERNAL_POSITION_GAIN = config["EXTERNAL_POSITION_GAIN"].as<float>();
        EXTERNAL_ROTATION_GAIN = config["EXTERNAL_ROTATION_GAIN"].as<float>();

        CAL_FINISH_FORCE_SIZE = config["CAL_FINISH_FORCE_SIZE"].as<float>();
        CAL_FINISH_TORQUE_SIZE = config["CAL_FINISH_TORQUE_SIZE"].as<float>();

        EDGE_AVERAGE_STOCK_SIZE = config["EDGE_AVERAGE_STOCK_SIZE"].as<int>();
        EDGE_FIXED_START_POINT_VARIANCE_THRESHOLD = config["EDGE_FIXED_START_POINT_VARIANCE_THRESHOLD"].as<float>();
        EDGE_FIXED_DIRECTION_VARIANCE_THRESHOLD = config["EDGE_FIXED_DIRECTION_VARIANCE_THRESHOLD"].as<float>();
        FIXED_EDGE_RATIO_THRESHOLD = config["FIXED_EDGE_RATIO_THRESHOLD"].as<float>();

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
        if (!calculate_frame_pose(edge_nodes_to_calculate, Pose3D(), false, current_frame_pose)) {
            continue;
        }

        // check if the calculated frame_pose is valid
        std::vector<float> translation_stress_with_calculated_edges, rotation_stress_with_calculated_edges;
        get_stress(edge_nodes_to_calculate,
                   current_frame_pose,
                   translation_stress_with_calculated_edges,
                   rotation_stress_with_calculated_edges);

        // TODO: maybe this is wrong. we need to frame_pose_is_valid = true.
        //if (frame_pose.translationalDiffTo(current_frame_pose).norm() < FRAME_POSE_CAL_FINISH_TRANSLATIONAL_DELTA &&
        //    frame_pose.rotationalDiffTo(current_frame_pose).norm() < FRAME_POSE_CAL_FINISH_ROTATIONAL_DELTA) {
        //    current_frame_pose.copy_to(frame_pose);
        //    break;
        //}

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
        frame_pose_is_valid = true;

        current_frame_pose.copy_to(frame_pose);
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

    VectorAverage translation_force_average(1); // TODO: remove this line
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

    // TODO: change the condition no to use CAL_FINISH_FORCE_SIZE and CAL_FINISH_TORQUE_SIZE
    if ((force_to_edge_moved.force * EDGE_POSE_TRANSLATE_GAIN).cwiseAbs2().sum() < CAL_FINISH_FORCE_SIZE &&
        (force_to_edge_moved.torque * EDGE_POSE_ROTATE_GAIN).cwiseAbs2().sum() < CAL_FINISH_TORQUE_SIZE) {
        return false;
    }

    edge_id = set_edge3d(edge.start_point(),
                         edge.direction(),
                         edge.length());
    return true;
}

bool EdgeSpaceDynamics::optimize(Pose3D& frame_pose,
                                 std::vector<EdgeNode>& edge_nodes,
                                 const Pose3D& extarnal_pose_data,
                                 const bool use_external_pose_data) {

    // edge_node as invalid if the edge is not exist
    for (int i = 0; i < edge_nodes.size(); i++) {
        try {
            get_edge_index(edge_nodes[i].edge_id);
        } catch (std::invalid_argument& e) {
            edge_nodes[i].is_valid = false;
        }
    }

    bool is_frame_pose_fixed = false;
    if (!update_dynamics(edge_nodes, extarnal_pose_data, true, true, use_external_pose_data, is_frame_pose_fixed, frame_pose)) {
        std::cout << "EdgeSpaceDynamics::optimize: failed to update frame pose." << std::endl;
        return false;
    }

    if (edge_nodes.size() <= EDGE_NUM_TO_GET_FRAME_POSE) return true;

    if (!is_frame_pose_fixed) return true;

    float fixed_edges_ratio = fixed_edge_ratio(edge_nodes);

    // remove less updated edge line.
    if (fixed_edges_ratio == 1) {
        if (joint_edge_3d(edge_nodes)) return true;

        // TODO: need to remove edge point from all key frames
        if (remove_less_updated_edge(edge_nodes)) return true;
    }

    // find invalid edge nodes if fixed edges ratio is less than the threshold
    else if (fixed_edges_ratio > FIXED_EDGE_RATIO_THRESHOLD) {
        check_invalid_edge_nodes(edge_nodes);
    }

    return true;
}

// frame_pose should be fixed before calling this function
bool EdgeSpaceDynamics::find_invalid_edge_nodes(const Pose3D& frame_pose,
                                                std::vector<EdgeNode>& edge_nodes) {

    if (edge_nodes.size() < EDGE_NUM_TO_GET_FRAME_POSE) {
        std::cout << "EdgeSpaceDynamics::find_invalid_edge_nodes: edge_nodes.size() < EDGE_NUM_TO_GET_FRAME_POSE" << std::endl;
        return false;
    }

    // remove most stressfull node
    std::vector<float> translation_stress, rotation_stress;
    get_stress(edge_nodes, frame_pose, translation_stress, rotation_stress);
    float max_translation_stress = *std::max_element(translation_stress.begin(), translation_stress.end());
    int max_translation_stress_index = std::distance(translation_stress.begin(), std::max_element(translation_stress.begin(), translation_stress.end()));

    float translation_stress_threshold = max_translation_stress * 0.4; // TODO: set parameter

    // set the max_translation_stress_index as invalid if the second max stress is less than the threshold
    translation_stress[max_translation_stress_index] = 0;
    float second_max_translation_stress = *std::max_element(translation_stress.begin(), translation_stress.end());
    if (second_max_translation_stress < translation_stress_threshold) {
        edge_nodes[max_translation_stress_index].is_valid = false;
    }

    return true;
}

std::vector<Line3D> EdgeSpaceDynamics::get_edge3ds() {
    return edges;
}

Line3D EdgeSpaceDynamics::get_edge3d(int edge_id) {
    try {
        return edges[get_edge_index(edge_id)];
    } catch (std::invalid_argument& e) {
        throw std::invalid_argument("EdgeSpaceDynamics::get_edge3d: edge id is not exist. id: " + std::to_string(edge_id));
        return Line3D(0, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0), 0);
    }
}

void EdgeSpaceDynamics::remove_edge3d(int edge_id) {
    int index = get_edge_index(edge_id);
    edges.erase(edges.begin() + index);
    edge_ids.erase(edge_ids.begin() + index);
}

int EdgeSpaceDynamics::set_edge3d(Eigen::Vector3f start_point,
                                  Eigen::Vector3f direction,
                                  float length) {
    // edge id is the max element of the edge_ids + 1
    int edge_id = edge_ids.size() == 0 ? 0 : *std::max_element(edge_ids.begin(), edge_ids.end()) + 1;

    Line3D edge3d(edge_id,
                  start_point,
                  direction,
                  length,
                  EDGE_AVERAGE_STOCK_SIZE,
                  EDGE_FIXED_START_POINT_VARIANCE_THRESHOLD,
                  EDGE_FIXED_DIRECTION_VARIANCE_THRESHOLD);

    // TODO: guarantee the edge is added to the edges and edge_ids only here
    // TODO: guarantee edge_id is unique
    edges.push_back(edge3d);
    edge_ids.push_back(edge_id);
    return edge_id;
}

int EdgeSpaceDynamics::set_edge3d(const EdgeNode edge_node,
                                  const Pose3D frame_pose,
                                  const float distance_to_edge) {
    return set_edge3d(frame_pose.position + frame_pose.rotateVectorToWorld(edge_node.direction_frame_to_edge) * distance_to_edge,
                      frame_pose.rotateVectorToWorld(Eigen::Vector3f(edge_node.edge_direction[0], edge_node.edge_direction[1], 0)),
                      0.0f);
}

bool EdgeSpaceDynamics::calculate_frame_pose(std::vector<EdgeNode> edge_nodes,
                                             const Pose3D& extarnal_pose_data,
                                             const bool use_external_pose_data,
                                             Pose3D& frame_pose) {

    // TODO: reconsider add rotation_average or not
    for (int i = 0; i < MAX_CAL_ITER; i++) {
        bool is_frame_pose_fixed = false;
        if (!update_dynamics(edge_nodes, extarnal_pose_data, false, true, use_external_pose_data, is_frame_pose_fixed, frame_pose)) {
            return false;
        }

        if (is_frame_pose_fixed) {
            return true;
        }
    }
    return false;
}

void EdgeSpaceDynamics::get_stress(std::vector<EdgeNode> edge_nodes,
                                   Pose3D frame_pose,
                                   std::vector<float>& translation_stress,
                                   std::vector<float>& rotation_stress) {
    for (auto edge_node : edge_nodes) {
        if (!edge_node.is_valid) continue;
        Line3D edge = edges[get_edge_index(edge_node.edge_id)];

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


bool EdgeSpaceDynamics::update_dynamics(std::vector<EdgeNode> edge_nodes,
                                        const Pose3D& extarnal_pose_data,
                                        const bool update_edges,
                                        const bool update_frame_pose,
                                        const bool use_external_pose_data,
                                        bool& is_frame_pose_fixed,
                                        Pose3D& frame_pose) {

    // to check if the frame_pose is fixed
    Pose3D latest_frame_pose = frame_pose.clone();

    // TODO: remove force_to_edge_sum, it's not used
    Force3D force_to_frame_sum, force_to_edge_sum;

    for (int j = 0; j < edge_nodes.size(); j++) {
        if (!edge_nodes[j].is_valid) continue;

        EdgeNode edge_node = edge_nodes[j];

        int edge_index = get_edge_index(edge_node.edge_id);

        Line3D edge = edges[edge_index];
    
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

        if (update_edges) {
            edges[edge_index].add_force(force_to_edge.force * EDGE_POSE_TRANSLATE_GAIN,
                                               force_to_edge.torque * EDGE_POSE_ROTATE_GAIN,
                                               torque_center_point_for_edge_line);
        }
    }

    if (update_frame_pose) {
        frame_pose.translate(force_to_frame_sum.force * FRAME_POSE_TRANSLATE_GAIN / edge_nodes.size());
        frame_pose.rotate(force_to_frame_sum.torque * FRAME_POSE_ROTATE_GAIN / edge_nodes.size());
    }

    if (use_external_pose_data && update_frame_pose) {
        Eigen::Vector3f translation_diff = frame_pose.translationalDiffTo(extarnal_pose_data);
        Eigen::Vector3f rotation_diff = frame_pose.rotationalDiffTo(extarnal_pose_data);

        frame_pose.translate(translation_diff * EXTERNAL_POSITION_GAIN);
        frame_pose.rotate(rotation_diff * EXTERNAL_ROTATION_GAIN);
    }

    // check if the frame_pose is fixed
    is_frame_pose_fixed = frame_pose.translationalDiffTo(latest_frame_pose).norm() < FRAME_POSE_CAL_FINISH_TRANSLATIONAL_DELTA &&
                         frame_pose.rotationalDiffTo(latest_frame_pose).norm() < FRAME_POSE_CAL_FINISH_ROTATIONAL_DELTA;

    return true;
}

int EdgeSpaceDynamics::get_edge_index(int edge_id) {
    int index = std::distance(edge_ids.begin(), std::find(edge_ids.begin(), edge_ids.end(), edge_id));

    // throw error if id is not exist
    if (index == edge_ids.size()) {
        throw std::invalid_argument("EdgeSpaceDynamics::get_edge_index: edge id is not exist.");
    }

    return index;
}

float EdgeSpaceDynamics::fixed_edge_ratio(std::vector<EdgeNode> edge_nodes) {
    int fixed_edges_count = 0;
    for (auto edge_node : edge_nodes) {
        if (get_edge3d(edge_node.edge_id).is_fixed()) {
            fixed_edges_count++;
        }
    }
    return (float)fixed_edges_count / (float)edge_nodes.size();
}

bool EdgeSpaceDynamics::remove_less_updated_edge(std::vector<EdgeNode>& edge_nodes) {
    int min_updated_count = std::numeric_limits<int>::max();
    int min_updated_edge_node_index = 0;
    float average_updated_count = 0;

    for (int i = 0; i < edge_nodes.size(); i++) {
        if (!edge_nodes[i].is_valid) continue;
        int edge_index = get_edge_index(edge_nodes[i].edge_id);
        int updated_count = edges[edge_index].update_count();
        average_updated_count += float(updated_count) / float(edge_nodes.size());
        if (updated_count < min_updated_count) {
            min_updated_count = updated_count;
            min_updated_edge_node_index = i;
        }
    }

    float threshold = average_updated_count * 0.5; // TODO: set parameter
    if (min_updated_count < threshold) {
        clear_edges_history(edge_nodes);
        edge_nodes[min_updated_edge_node_index].is_valid = false;
        remove_edge3d(edge_nodes[min_updated_edge_node_index].edge_id);
        return true;
    }

    return false;
}

bool EdgeSpaceDynamics::joint_edge_3d(std::vector<EdgeNode>& edge_nodes) {
    bool did_joint = false;
    for (int i = 0; i < edge_nodes.size(); i++) {
        if (!edge_nodes[i].is_valid) continue;
        int edge_index = get_edge_index(edge_nodes[i].edge_id);

        if (!edges[edge_index].is_fixed()) continue;

        for (int j = 0; j < edge_nodes.size(); j++) {

            if (i == j) continue;

            if (edge_nodes[i].edge_id == edge_nodes[j].edge_id) continue;

            if (!edge_nodes[j].is_valid) continue;
            int edge_index2 = get_edge_index(edge_nodes[j].edge_id);

            if (!edges[edge_index2].is_fixed()) continue;

            if (edges[edge_index].connect(edges[edge_index2])) {
                clear_edges_history(edge_nodes);

                std::cout << "jointed id: " << edge_nodes[j].edge_id << " to " << edge_nodes[i].edge_id << std::endl;
                edge_nodes[j].edge_id = edge_nodes[i].edge_id;
                did_joint = true;
            }
        }
    }

    return did_joint;
}

void EdgeSpaceDynamics::check_invalid_edge_nodes(std::vector<EdgeNode>& edge_nodes) {
    for (int i = 0; i < edge_nodes.size(); i++) {
        if (!edge_nodes[i].is_valid) continue;
        if (!edges[get_edge_index(edge_nodes[i].edge_id)].is_fixed()) {
            edge_nodes[i].is_valid = false;
        }

        edges[get_edge_index(edge_nodes[i].edge_id)].clear_history();
    }
}

void EdgeSpaceDynamics::clear_edges_history(std::vector<EdgeNode>& edge_nodes) {
    for (int i = 0; i < edge_nodes.size(); i++) {
        if (!edge_nodes[i].is_valid) continue;
        edges[get_edge_index(edge_nodes[i].edge_id)].clear_history();
    }
}
