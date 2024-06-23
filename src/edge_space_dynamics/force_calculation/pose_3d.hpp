#ifndef POSE_3D_HPP
#define POSE_3D_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

struct Pose3D {
public:
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;

    Pose3D() : position(0.0f, 0.0f, 0.0f), orientation(Eigen::Quaternionf::Identity()) {}

    Pose3D(Eigen::Vector3f position, Eigen::Quaternionf orientation)
        : position(position), orientation(orientation) {}

    void translate(const Eigen::Vector3f& delta) {
        position += delta;
    }

    void rotate(const Eigen::Vector3f& rotate_vector) {
        Eigen::Quaternionf q;
        q = Eigen::AngleAxisf(rotate_vector.norm(), rotate_vector.normalized());
        orientation = q * orientation;
    }

    Eigen::Matrix4f getTransformationMatrix() const {
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
        transformation.block<3,3>(0,0) = orientation.toRotationMatrix();
        transformation.block<3,1>(0,3) = position;
        return transformation;
    }

    Eigen::Vector3f transformToWorld(const Eigen::Vector3f& point) const {
        return (orientation * point) + position;
    }

    Eigen::Vector3f transformToLocal(const Eigen::Vector3f& point) const {
        return orientation.inverse() * (point - position);
    }

    Eigen::Vector3f rotateVectorToWorld(const Eigen::Vector3f& vector) const {
        return orientation * vector;
    }

    Eigen::Vector3f rotateVectorToLocal(const Eigen::Vector3f& vector) const {
        return orientation.inverse() * vector;
    }

    // Function to return a copy of this instance
    Pose3D clone() const {
        Pose3D copy;
        copy.position = this->position;
        copy.orientation = this->orientation;
        return copy;
    }

    void copy_to(Pose3D& other) const {
        other.position = this->position;
        other.orientation = this->orientation;
    }
};

#endif // POSE_3D_HPP

