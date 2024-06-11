#include "force_calculation.hpp"

// TODO: make this as class.
bool force_calculation(const Line3D &edge,
                       const EdgeNode &edge_node,
                       const Pose3D &frame_pose,
                       Force3D &force_to_frame,
                       Force3D &force_to_edge,
                       float &torque_center_point_for_edge_line) {

    Line3D observed_line(-1, // do not care about id
                         frame_pose.position,
                         frame_pose.rotateVectorToWorld(edge_node.direction_frame_to_edge),
                         1.0f);

    // Get the closest points between the edge and the frame-to-edge line
    float distance_edge, distance_observed_line;
    bool result = Line3D::get_closest_points_between(edge, observed_line, distance_edge, distance_observed_line);

    if (!result) {
        return false;
    }

    // Get the vector from the closest point on the observed line to the edge
    // this will be the force
    Eigen::Vector3f force = edge.get_point_at(distance_edge) - observed_line.get_point_at(distance_observed_line);

    // Get the vector from the frame to the closest point on the edge
    Eigen::Vector3f vector_frame_to_edge = edge.get_point_at(distance_edge) - frame_pose.position;
    // calculate cross product of observed_line.direction and vector_frame_to_edge
    // this will be the torque
    Eigen::Vector3f torque = observed_line.direction().cross(vector_frame_to_edge) / (vector_frame_to_edge.norm() * observed_line.direction().norm());


    // Calculate the torque to adjust the direction of edge line and observed line
    // z-axis of observed_line_centered_pose is same as observed line direction
    Pose3D observed_line_centered_pose = frame_pose.clone();
    float x_rotate_angle = atan2(edge_node.direction_frame_to_edge(1), edge_node.direction_frame_to_edge(2));
    float y_rotate_angle = atan2(edge_node.direction_frame_to_edge(0), edge_node.direction_frame_to_edge(2));

    observed_line_centered_pose.rotate(observed_line_centered_pose.rotateVectorToWorld(Eigen::Vector3f(1.0f, 0.0f, 0.0f)) * x_rotate_angle);
    observed_line_centered_pose.rotate(observed_line_centered_pose.rotateVectorToWorld(Eigen::Vector3f(0.0f, 1.0f, 0.0f)) * y_rotate_angle);

    Eigen::Vector3f observed_line_centered_edge_direction = observed_line_centered_pose.rotateVectorToLocal(edge.direction());
    Eigen::Vector2f observed_line_centered_edge_direction_2d(observed_line_centered_edge_direction(0), observed_line_centered_edge_direction(1));
    // calculate cross product between observed_line_centered_edge_direction_2d and edge_node.edge_direction
    float sin_angle = edge_node.edge_direction(0) * observed_line_centered_edge_direction_2d(1) - edge_node.edge_direction(1) * observed_line_centered_edge_direction_2d(0);
    sin_angle = sin_angle / (observed_line_centered_edge_direction_2d.norm() * edge_node.edge_direction.norm());

    // torque is angle * observed_line.direction
    Eigen::Vector3f torque_to_adjust = sin_angle * observed_line.direction();
    torque_to_adjust(1) = -torque_to_adjust(1); // y-axis is flipped

    force_to_frame = Force3D(force, torque + torque_to_adjust);
    force_to_edge = Force3D(-force, - torque_to_adjust);
    torque_center_point_for_edge_line = distance_edge;

    return true;
}
