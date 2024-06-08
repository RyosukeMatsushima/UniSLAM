#ifndef EDGE_SPACE_DYNAMICS_HPP
#define EDGE_SPACE_DYNAMICS_HPP

#define MAX_CAL_ITER 1000
#define CAL_FINISH_FORCE_SIZE 0.0001
#define CAL_FINISH_TORQUE_SIZE 0.0001

#define FRAME_POSE_TRANSLATE_GAIN 0.1
#define FRAME_POSE_ROTATE_GAIN -0.1

#include <Eigen/Dense>

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
//    void double_frame_init(); // this should be same as add_new_edge

    // usecase1
    // Calculate the pose of the new frame
    // The new frame should be related to the calculated edges
    bool get_frame_pose(std::vector<EdgeNode> edge_nodes,
                        Pose3D& frame_pose);

    // usecase2
    // Add new edge
    // returns the edge pointer
    // returns -1 if the edge is not added
    bool add_new_edge(Pose3D frame1_pose,
                      Pose3D frame2_pose,
                      EdgeNode frame1_edge_node,
                      EdgeNode frame2_edge_node,
                      int& edge_id);


    // usecase5
    // returns the optimized frame_pose
    // optimize the 3d edge
    Pose3D optimize(Pose3D frame_pose,
                    std::vector<EdgeNode> edge_nodes);

    // usecase3
    void get_edge3ds(std::vector<Line3D>& edge3ds);

    // usecase4
    // returns id of the edge
    int set_edge3d(Eigen::Vector3f start_point,
                   Eigen::Vector3f direction,
                   float length);

private:

    std::vector<Line3D> edges;
    std::vector<int> edge_ids;
};

#endif // EDGE_SPACE_DYNAMICS_HPP

