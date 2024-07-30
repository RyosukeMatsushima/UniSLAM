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
};

TEST_F(LocalSlamTest, withSquareSpaceWithoutExternalPose) {
    float dxy_position = 0.1;

    // initialize local_slam
    // initialize should not finish without movement
    ASSERT_FALSE(doMultiFrameInit());

    // move position x-axis
    movePosition(dxy_position, 0, 0);
    // initialize should not finish with only x-axis movement
    ASSERT_FALSE(doMultiFrameInit());

    // move position y-axis
    movePosition(0, dxy_position, 0);
    // initialize should finish
    ASSERT_TRUE(doMultiFrameInit());

    // back to original position
    movePosition(-dxy_position, -dxy_position, 0);
    Pose3D pose;
    ASSERT_TRUE(doUpdate(pose));

    // check the pose
    Eigen::Vector3f expected_position(0, 0, 0);
    Eigen::Quaternionf expected_orientation(1, 0, 0, 0);

    float allowed_error = 0.001;
    ASSERT_NEAR(pose.position.x(), expected_position.x(), allowed_error);
    ASSERT_NEAR(pose.position.y(), expected_position.y(), allowed_error);
    ASSERT_NEAR(pose.position.z(), expected_position.z(), allowed_error);

    ASSERT_NEAR(pose.orientation.x(), expected_orientation.x(), allowed_error);
    ASSERT_NEAR(pose.orientation.y(), expected_orientation.y(), allowed_error);
    ASSERT_NEAR(pose.orientation.z(), expected_orientation.z(), allowed_error);
    ASSERT_NEAR(pose.orientation.w(), expected_orientation.w(), allowed_error);

    // TODO: check pose with more movement. Need to allow the position is scaled without external pose data.
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

