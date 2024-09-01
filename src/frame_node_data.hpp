#ifndef FRAME_NODE_DATA_HPP
#define FRAME_NODE_DATA_HPP

#include "frame_node.hpp"
#include "pose_3d.hpp"

struct FrameNodeData {
    FrameNode frame_node;
    Pose3D external_pose_data;
    Pose3D calculated_pose;
    bool use_external_pose_data;

    FrameNodeData(const FrameNode& frame_node,
                  const Pose3D& external_pose_data,
                  const Pose3D& calculated_pose,
                  const bool use_external_pose_data)
        : frame_node(frame_node),
          external_pose_data(external_pose_data),
          calculated_pose(calculated_pose),
          use_external_pose_data(use_external_pose_data) {}
};
#endif // FRAME_NODE_DATA_HPP
