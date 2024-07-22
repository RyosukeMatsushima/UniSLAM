#include "force_calculation.hpp"

ForceCalculation::ForceCalculation(const Line3D &edge, const EdgeNode &edge_node, const Pose3D &frame_pose)
    : edge(edge), edge_node(edge_node), frame_pose(frame_pose) {}

bool ForceCalculation::calculate() {
    Line3D observed_line = createObservedLine();

    float distance_edge, distance_observed_line;
    if (!calculateClosestPoints(observed_line, distance_edge, distance_observed_line)) {
        return false;
    }

    Eigen::Vector3f force = calculateForce(observed_line, distance_edge, distance_observed_line);
    Eigen::Vector3f torque = calculateTorque(observed_line, distance_edge);
    Eigen::Vector3f torque_to_adjust = calculateTorqueToAdjust(observed_line);

    force_to_frame = Force3D(force, torque + torque_to_adjust);
    force_to_edge = Force3D(-force, -torque_to_adjust);
    torque_center_point_for_edge_line = distance_edge;

    return true;
}

Force3D ForceCalculation::getForceToFrame() const {
    return force_to_frame;
}

Force3D ForceCalculation::getForceToEdge() const {
    return force_to_edge;
}

float ForceCalculation::getTorqueCenterPointForEdgeLine() const {
    return torque_center_point_for_edge_line;
}

Line3D ForceCalculation::createObservedLine() const {
    return Line3D(-1, // do not care about id
                  frame_pose.position,
                  frame_pose.rotateVectorToWorld(edge_node.direction_frame_to_edge),
                  1.0f);
}

bool ForceCalculation::calculateClosestPoints(const Line3D &observed_line, float &distance_edge, float &distance_observed_line) const {
    return Line3D::get_closest_points_between(edge, observed_line, distance_edge, distance_observed_line);
}

Eigen::Vector3f ForceCalculation::calculateForce(const Line3D &observed_line, float distance_edge, float distance_observed_line) const {
    return edge.get_point_at(distance_edge) - observed_line.get_point_at(distance_observed_line);
}

Eigen::Vector3f ForceCalculation::calculateTorque(const Line3D &observed_line, float distance_edge) const {
    Eigen::Vector3f vector_frame_to_edge = edge.get_point_at(distance_edge) - frame_pose.position;
    return observed_line.direction().cross(vector_frame_to_edge) / (vector_frame_to_edge.norm() * observed_line.direction().norm());
}

Eigen::Vector3f ForceCalculation::calculateTorqueToAdjust(const Line3D &observed_line) const {
    Pose3D observed_line_centered_pose = frame_pose.clone();
    float x_rotate_angle = atan2(edge_node.direction_frame_to_edge(1), edge_node.direction_frame_to_edge(2));
    float y_rotate_angle = atan2(edge_node.direction_frame_to_edge(0), edge_node.direction_frame_to_edge(2));

    observed_line_centered_pose.rotate(observed_line_centered_pose.rotateVectorToWorld(Eigen::Vector3f(1.0f, 0.0f, 0.0f)) * x_rotate_angle);
    observed_line_centered_pose.rotate(observed_line_centered_pose.rotateVectorToWorld(Eigen::Vector3f(0.0f, 1.0f, 0.0f)) * y_rotate_angle);

    Eigen::Vector3f observed_line_centered_edge_direction = observed_line_centered_pose.rotateVectorToLocal(edge.direction());
    Eigen::Vector2f observed_line_centered_edge_direction_2d(observed_line_centered_edge_direction(0), observed_line_centered_edge_direction(1));
    float sin_angle = edge_node.edge_direction(0) * observed_line_centered_edge_direction_2d(1) - edge_node.edge_direction(1) * observed_line_centered_edge_direction_2d(0);
    sin_angle = sin_angle / (observed_line_centered_edge_direction_2d.norm() * edge_node.edge_direction.norm());

    Eigen::Vector3f torque_to_adjust = sin_angle * observed_line.direction();
    // TODO: It is not clear why the y-axis is flipped here
    torque_to_adjust(1) = -torque_to_adjust(1); // y-axis is flipped

    return torque_to_adjust;
}

