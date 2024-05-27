#ifndef POSE_3D_HPP
#define POSE_3D_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

class Pose3D {
public:
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;

    Pose3D() : position(0.0f, 0.0f, 0.0f), orientation(Eigen::Quaternionf::Identity()) {}

    void translate(const Eigen::Vector3f& delta) {
        position += delta;
    }

    void rotate(const Eigen::Vector3f& axis, float angle) {
        Eigen::AngleAxisf rotation(angle, axis.normalized());
        orientation = rotation * orientation;
        orientation.normalize();
    }

    Eigen::Matrix4f getTransformationMatrix() const {
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
        transformation.block<3,3>(0,0) = orientation.toRotationMatrix();
        transformation.block<3,1>(0,3) = position;
        return transformation;
    }
};

#endif // POSE_3D_HPP
