#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "local_slam.hpp"
#include "double_squares_space.hpp"
#include "camera_model.hpp"
#include "debug_view.hpp"

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
                      local_slam(CameraModel(double_squares_space.getCameraMatrix(), double_squares_space.getImageSize())) {}

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
    float dxy_position = 0.2;

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

TEST_F(LocalSlamTest, fixEdges) {
    float dxy_position = 0.03;

    int window_size = 20;
    float angle_resolution = 0.2f;

    cv::Mat image1 = getCurrentImage();
    FrameNode frame_node1(image1, window_size, angle_resolution);
    Pose3D pose1 = getCurrentPose();

    std::cout << "pose1: " << pose1.position.transpose() << std::endl;

    movePosition(0, dxy_position, 0);
    cv::Mat image2 = getCurrentImage();
    FrameNode frame_node2(image2, window_size, angle_resolution);
    Pose3D pose2 = getCurrentPose();

    local_slam.fix_edges(frame_node1, frame_node2, pose1, pose2);

    std::cout << "pose2: " << pose2.position.transpose() << std::endl;

    std::cout << "image1.size(): " << image1.size() << std::endl;
    std::cout << "image1.type(): " << image1.type() << std::endl;

    DebugView debug_view(image1);

    std::cout << "frame_node1.getFixedEdgePoints().size(): " << frame_node1.getFixedEdgePoints().size() << std::endl;
    debug_view.drawEdgePoints(frame_node1.getFixedEdgePoints());
    std::cout << "frame_node2.getFixedEdgePoints().size(): " << frame_node2.getFixedEdgePoints().size() << std::endl;
    cv::imwrite(RESULT_IMAGE_PATH "fix_edges_from_pose1.png", debug_view.getDebugImage());



    DebugView debug_view2(image2);
    debug_view2.drawEdgePoints(frame_node2.getFixedEdgePoints());
    cv::imwrite(RESULT_IMAGE_PATH "fix_edges_from_pose2.png", debug_view2.getDebugImage());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

