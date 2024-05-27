#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "pose_3d.hpp"


// Test default constructor
TEST(Pose3DTest, DefaultConstructor) {
    Pose3D pose;
    EXPECT_EQ(pose.position, Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_TRUE(pose.orientation.isApprox(Eigen::Quaternionf::Identity()));
}

// Test translation
TEST(Pose3DTest, Translate) {
    Pose3D pose;
    pose.translate(Eigen::Vector3f(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(pose.position, Eigen::Vector3f(1.0f, 2.0f, 3.0f));
}

// Test rotation
TEST(Pose3DTest, Rotate) {
    Pose3D pose;
    Eigen::Vector3f axis(0.0f, 1.0f, 0.0f);
    float angle = M_PI; // 180 degrees in radians
    pose.rotate(axis, angle);

    Eigen::Quaternionf expectedOrientation = Eigen::Quaternionf(0.0f, 0.0f, 1.0f, 0.0f);

    EXPECT_NEAR(pose.orientation.x(), expectedOrientation.x(), 1e-6);
    EXPECT_NEAR(pose.orientation.y(), expectedOrientation.y(), 1e-6);
    EXPECT_NEAR(pose.orientation.z(), expectedOrientation.z(), 1e-6);
    EXPECT_NEAR(pose.orientation.w(), expectedOrientation.w(), 1e-6);
}

// Test transformation matrix
TEST(Pose3DTest, TransformationMatrix) {
    Pose3D pose;
    pose.translate(Eigen::Vector3f(1.0f, 2.0f, 3.0f));
    pose.rotate(Eigen::Vector3f(0.0f, 1.0f, 0.0f), M_PI / 4); // 45 degrees in radians

    Eigen::Matrix4f transformation = pose.getTransformationMatrix();
    Eigen::Matrix4f expectedTransformation = Eigen::Matrix4f::Identity();
    expectedTransformation.block<3,1>(0,3) = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
    Eigen::AngleAxisf rotation(M_PI / 4, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
    expectedTransformation.block<3,3>(0,0) = rotation.toRotationMatrix();

    EXPECT_TRUE(transformation.isApprox(expectedTransformation));
}
