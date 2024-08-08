#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "local_slam.hpp"
#include "double_squares_space.hpp"
#include "camera_model.hpp"
#include "debug_view.hpp"

#define CONFIG_FILE_PATH PROJECT_SOURCE_DIR "/config_for_test/"
#define RESULT_IMAGE_PATH PROJECT_SOURCE_DIR "/test/result/"

class LocalSlamTest : public ::testing::Test {
protected:
    DoubleSquaresSpace double_squares_space;
    LocalSlam local_slam;

    cv::Mat position = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    cv::Mat rotation = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                                                  0, 1, 0,
                                                  0, 0, 1);

    LocalSlamTest() : double_squares_space(),
                      local_slam(CameraModel(double_squares_space.getCameraMatrix(), double_squares_space.getImageSize()),
                                 CONFIG_FILE_PATH "edge_space_dynamics.yaml") {}

    void movePosition(float x, float y, float z) {
        position.at<double>(0) += double(x);
        position.at<double>(1) += double(y);
        position.at<double>(2) += double(z);
    }

    bool doMultiFrameInit() {
        cv::Mat current_image = getCurrentImage();
        return local_slam.multi_frame_init(current_image, getPosition(), getOrientation());
    }

    Eigen::Vector3f getPosition() {
        return Eigen::Vector3f(float(position.at<double>(0)), float(position.at<double>(1)), float(position.at<double>(2)));
    }

    Eigen::Quaternionf getOrientation() {
        Eigen::Matrix3f rotation_matrix;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                rotation_matrix(i, j) = float(rotation.at<double>(i, j));
            }
        }
        Eigen::Quaternionf q(rotation_matrix);
        return q;
    }

    cv::Mat getCurrentImage() {
        return double_squares_space.getImage(rotation, position);
    }

    Pose3D getCurrentPose() {
        return Pose3D(getPosition(), getOrientation());
    }

    bool doUpdate(Pose3D& pose) {
        cv::Mat current_image = getCurrentImage();
        return local_slam.update(current_image, pose);
    }

    void checkPose(Pose3D& estimated_pose) {
        Pose3D current_pose = getCurrentPose();

        float allowed_error = 0.01;
        ASSERT_NEAR(estimated_pose.position.x(), current_pose.position.x(), allowed_error);
        ASSERT_NEAR(estimated_pose.position.y(), current_pose.position.y(), allowed_error);
        ASSERT_NEAR(estimated_pose.position.z(), current_pose.position.z(), allowed_error);

        ASSERT_NEAR(estimated_pose.orientation.x(), current_pose.orientation.x(), allowed_error);
        ASSERT_NEAR(estimated_pose.orientation.y(), current_pose.orientation.y(), allowed_error);
        ASSERT_NEAR(estimated_pose.orientation.z(), current_pose.orientation.z(), allowed_error);
        ASSERT_NEAR(estimated_pose.orientation.w(), current_pose.orientation.w(), allowed_error);
    }
};

TEST_F(LocalSlamTest, withSquareSpaceWithoutExternalPose) {
    float dxy_position = 0.1;

    // initialize local_slam
    // initialize should not finish without movement
    ASSERT_FALSE(doMultiFrameInit());
    local_slam.save_log(RESULT_IMAGE_PATH);

    // move position x-axis
    movePosition(dxy_position, 0, 0);
    // initialize should not finish with only x-axis movement
    ASSERT_FALSE(doMultiFrameInit());
    local_slam.save_log(RESULT_IMAGE_PATH);

    // move position y-axis
    movePosition(0, dxy_position, 0);
    // initialize should finish
    ASSERT_TRUE(doMultiFrameInit());
    local_slam.save_log(RESULT_IMAGE_PATH);

    // check getPose
    Pose3D pose;
    EXPECT_TRUE(doUpdate(pose));
    local_slam.save_log(RESULT_IMAGE_PATH);
    checkPose(pose);

    // back to original position
    movePosition(-dxy_position, -dxy_position, 0);
    pose = Pose3D(Eigen::Vector3f(0.1, 0.1, 0), Eigen::Quaternionf::Identity());
    ASSERT_TRUE(doUpdate(pose));
    local_slam.save_log(RESULT_IMAGE_PATH);
    checkPose(pose);

    // TODO: check pose with more movement. Need to allow the position is scaled without external pose data.
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

