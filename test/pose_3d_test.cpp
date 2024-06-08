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
    pose.rotate(axis * angle);

    Eigen::Quaternionf expectedOrientation = Eigen::Quaternionf(0.0f, 0.0f, 1.0f, 0.0f);

    EXPECT_NEAR(pose.orientation.x(), expectedOrientation.x(), 1e-6);
    EXPECT_NEAR(pose.orientation.y(), expectedOrientation.y(), 1e-6);
    EXPECT_NEAR(pose.orientation.z(), expectedOrientation.z(), 1e-6);
    EXPECT_NEAR(pose.orientation.w(), expectedOrientation.w(), 1e-6);

    // rotate back and value should be same as intial pose
    pose.rotate(-axis * angle);
    Pose3D expected_pose;
    EXPECT_NEAR(pose.orientation.x(), expected_pose.orientation.x(), 1e-6);
    EXPECT_NEAR(pose.orientation.y(), expected_pose.orientation.y(), 1e-6);
    EXPECT_NEAR(pose.orientation.z(), expected_pose.orientation.z(), 1e-6);
    EXPECT_NEAR(pose.orientation.w(), expected_pose.orientation.w(), 1e-6);
}

// Test rotate back and check if the pose is same as initial pose
TEST(Pose3DTest, RotateBackAndForth) {
    Pose3D pose;
    Eigen::Vector3f axis(-0.0708944, 0.00457347, -0.0705402);
    float angle = 0.5f;
    pose.rotate(axis * angle);
    pose.rotate(-axis * angle);

    Pose3D expected_pose;
    EXPECT_NEAR(pose.orientation.x(), expected_pose.orientation.x(), 1e-6);
    EXPECT_NEAR(pose.orientation.y(), expected_pose.orientation.y(), 1e-6);
    EXPECT_NEAR(pose.orientation.z(), expected_pose.orientation.z(), 1e-6);
    EXPECT_NEAR(pose.orientation.w(), expected_pose.orientation.w(), 1e-6);
}

// Test transformation matrix
TEST(Pose3DTest, TransformationMatrix) {
    Pose3D pose;
    pose.translate(Eigen::Vector3f(1.0f, 2.0f, 3.0f));
    pose.rotate(Eigen::Vector3f(0.0f, 1.0f, 0.0f) * M_PI / 4); // 45 degrees in radians

    Eigen::Matrix4f transformation = pose.getTransformationMatrix();
    Eigen::Matrix4f expectedTransformation = Eigen::Matrix4f::Identity();
    expectedTransformation.block<3,1>(0,3) = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
    Eigen::AngleAxisf rotation(M_PI / 4, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
    expectedTransformation.block<3,3>(0,0) = rotation.toRotationMatrix();

    EXPECT_TRUE(transformation.isApprox(expectedTransformation));
}

TEST(Pose3DTest, TransformToWorld) {

    Pose3D pose;
    pose.translate(Eigen::Vector3f(1.0f, 2.0f, 3.0f));
    pose.rotate(Eigen::Vector3f(0.0f, 0.0f, 1.0f) * M_PI / 2); // 45 degrees in radians

    Eigen::Vector3f point(1.0f, 2.0f, 3.0f);
    Eigen::Vector3f transformedPoint = pose.transformToWorld(point);

    Eigen::Vector3f expectedTransformedPoint(-1.0f, 3.0f, 6.0f);
    EXPECT_TRUE(transformedPoint.isApprox(expectedTransformedPoint));
}

TEST(Pose3DTest, TransformToLocal) {
    Pose3D pose;
    pose.translate(Eigen::Vector3f(1.0f, 2.0f, 3.0f));
    pose.rotate(Eigen::Vector3f(0.0f, 0.0f, 1.0f) * M_PI / 2); // 90 degrees in radians

    Eigen::Vector3f point(-1.0f, 3.0f, 6.0f);
    Eigen::Vector3f transformedPoint = pose.transformToLocal(point);

    Eigen::Vector3f expectedTransformedPoint(1.0f, 2.0f, 3.0f);
    EXPECT_TRUE(transformedPoint.isApprox(expectedTransformedPoint));
}

TEST(Pose3DTest, RotateVectorToWorld) {
    Pose3D pose;
    pose.translate(Eigen::Vector3f(-1.0f, 13.0f, 3.0f)); // doesn't matter
    pose.rotate(Eigen::Vector3f(0.0f, 0.0f, 1.0f) * M_PI / 4); // 45 degrees in radians

    Eigen::Vector3f vector(1.0f, 0.0f, 0.0f);
    Eigen::Vector3f rotatedVector = pose.rotateVectorToWorld(vector);

    Eigen::Vector3f expectedRotatedVector(0.70710678118f, 0.70710678118f, 0.0f);
    EXPECT_TRUE(rotatedVector.isApprox(expectedRotatedVector));
}

TEST(Pose3DTest, RotateVectorToLocal) {
    Pose3D pose;
    pose.translate(Eigen::Vector3f(1.0f, 2.0f, 3.0f)); // doesn't matter
    pose.rotate(Eigen::Vector3f(0.0f, 0.0f, 1.0f) * M_PI / 4); // 45 degrees in radians

    Eigen::Vector3f vector(0.70710678118f, 0.70710678118f, 0.0f);
    Eigen::Vector3f rotatedVector = pose.rotateVectorToLocal(vector);

    Eigen::Vector3f expectedRotatedVector(1.0f, 0.0f, 0.0f);
    EXPECT_TRUE(rotatedVector.isApprox(expectedRotatedVector));
}

TEST(Pose3DTest, GetCopy) {
    Pose3D pose;
    pose.translate(Eigen::Vector3f(1.0f, 2.0f, 3.0f));
    pose.rotate(Eigen::Vector3f(0.0f, 0.0f, 1.0f) * M_PI / 4); // 45 degrees in radians

    Pose3D poseClone = pose.clone();

    EXPECT_TRUE(pose.position.isApprox(poseClone.position));
    EXPECT_TRUE(pose.orientation.isApprox(poseClone.orientation));

    // translate the copy and check if the original is not affected
    Eigen::Vector3f translation(1.0f, 2.0f, 3.0f);
    poseClone.translate(translation);
    EXPECT_TRUE(poseClone.position.isApprox(pose.position + translation));
}

