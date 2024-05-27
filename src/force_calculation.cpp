#include "force_calculation.hpp"

Force3D force_calculation(const Line3D &edge,
                          const EdgeNode &edge_node,
                          const Pose3D &pose) {
    return Force3D(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
}
