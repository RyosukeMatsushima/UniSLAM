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

    cv::Mat getImageFrom(const Pose3D& pose) {
        cv::Mat position = (cv::Mat_<double>(3, 1) << pose.position.x(), pose.position.y(), pose.position.z());
        Eigen::Matrix3f rotation_matrix = pose.orientation.toRotationMatrix();
        cv::Mat rotation = (cv::Mat_<double>(3, 3) << rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
                                                      rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
                                                      rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));
        return double_squares_space.getImage(rotation, position);
    }

    Pose3D getCurrentPose() {
        return Pose3D(getPosition(), getOrientation());
    }

    void checkPose(Pose3D& estimated_pose) {
        Pose3D current_pose = getCurrentPose();

        // TODO: check allowed error
        float allowed_error = 0.1;
        EXPECT_NEAR(estimated_pose.position.x(), current_pose.position.x(), allowed_error);
        EXPECT_NEAR(estimated_pose.position.y(), current_pose.position.y(), allowed_error);
        EXPECT_NEAR(estimated_pose.position.z(), current_pose.position.z(), allowed_error);

        EXPECT_NEAR(estimated_pose.orientation.x(), current_pose.orientation.x(), allowed_error);
        EXPECT_NEAR(estimated_pose.orientation.y(), current_pose.orientation.y(), allowed_error);
        EXPECT_NEAR(estimated_pose.orientation.z(), current_pose.orientation.z(), allowed_error);
        EXPECT_NEAR(estimated_pose.orientation.w(), current_pose.orientation.w(), allowed_error);
    }

    void save_log() {
        std::string file_name;
        VslamDebugView current_debug_view = local_slam.get_current_debug_view(file_name);
        cv::imwrite(RESULT_IMAGE_PATH + file_name, current_debug_view.getDebugImage());

        VslamDebugView key_frame_debug_view = local_slam.get_key_frame_debug_view(file_name);
        cv::imwrite(RESULT_IMAGE_PATH + file_name, key_frame_debug_view.getDebugImage());

        Pose3D camera_pose;
        camera_pose.translate(Eigen::Vector3f(0.7, -0.6, -1.1));
        camera_pose.rotate(Eigen::Vector3f(-0.7, -0.7, 0));
        cv::Mat base_image = getImageFrom(camera_pose);
        cv::Mat camera_matrix = double_squares_space.getCameraMatrix();
        VslamDebugView third_person_view = local_slam.get_third_person_view(camera_pose,
                                                                            base_image,
                                                                            camera_matrix,
                                                                            file_name);
        cv::imwrite(RESULT_IMAGE_PATH + file_name, third_person_view.getDebugImage());
    }
};

TEST_F(LocalSlamTest, multiFrameInitWithExternalPoseData) {

    const int optimize_iteration = 100;

    float dxy_position = 0.01;
    float max_xy_position = 0.2;

    // multi frame initialization
    bool did_finish_initilization = false;
    for (float x = position.at<double>(0); x < max_xy_position; x += dxy_position) {
        movePosition(dxy_position, 0, 0);
        did_finish_initilization = local_slam.multi_frame_init(getCurrentImage());
        save_log();
        if (did_finish_initilization) break;
    }
    ASSERT_TRUE(did_finish_initilization);

    // camera moves to x-axis positive direction
    // still calculate pose should be floated and update() should return false
    for (float x = position.at<double>(0); x < max_xy_position; x += dxy_position) {
        movePosition(dxy_position, 0, 0);
        Pose3D calculateed_pose;
        local_slam.update(getCurrentImage(), getCurrentPose(), true, false, calculateed_pose);
        local_slam.optimize(optimize_iteration);
        save_log();
    }

    // camera moves to y-axis positive direction
    // before end of this movement, the calculated pose should be fixed and update() should return true
    for (float y = position.at<double>(1); y < max_xy_position; y += dxy_position) {
        movePosition(0, dxy_position, 0);
        Pose3D calculateed_pose;
        local_slam.update(getCurrentImage(), getCurrentPose(), true, false, calculateed_pose);
        local_slam.optimize(optimize_iteration);
        save_log();
    }

    for (int i = 0; i < 50; i++) {
        std::cout << "optimize iteration: " << i << std::endl;
        Pose3D calculateed_pose;
        local_slam.update(getCurrentImage(), getCurrentPose(), true, false, calculateed_pose);
        local_slam.optimize(optimize_iteration);
        save_log();
    }

    // check calculated pose
    for (float x = position.at<double>(0); x > -max_xy_position; x -= dxy_position) {
        std::cout << "calculate pose" << std::endl;
        movePosition(-dxy_position, -dxy_position, 0);
        Pose3D calculateed_pose;
        EXPECT_TRUE(local_slam.update(getCurrentImage(), Pose3D(), false, true, calculateed_pose));
        checkPose(calculateed_pose);
        save_log();
        local_slam.optimize(optimize_iteration);
    }

    // TODO: check pose with more movement. Need to allow the position is scaled without external pose data.
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

