#include <gtest/gtest.h>
#include "local_slam.hpp"
#include "polygons_space.hpp"

class LocalSlamTest : public ::testing::Test {
protected:
    LocalSlam local_slam;
    PolygonsSpace polygons_space;

    int img_width = 500;
    int img_height = 500;

    cv::Mat position = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    cv::Mat rotation = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                                                  0, 1, 0,
                                                  0, 0, 1);

    void SetUp() override {

        // setup polygons_space
        cv::Point3f p1(-1, 1, 0);
        cv::Point3f p2(1, 1, 0);
        cv::Point3f p3(1, 1, 1);
        cv::Point3f p4(-1, 1, 1);

        cv::Point3f p5(-1, -1, 0);
        cv::Point3f p6(1, -1, 0);
        cv::Point3f p7(1, -1, 1);
        cv::Point3f p8(-1, -1, 1);

        std::vector<cv::Point3f> polygon1 = {p1, p2, p3, p4};
        std::vector<cv::Point3f> polygon2 = {p5, p6, p7, p8};

        polygons_space.setPolygon(polygon1, cv::Scalar(0, 0, 255));
        polygons_space.setPolygon(polygon2, cv::Scalar(0, 255, 0));

    }

    cv::Mat getCameraMatrix() {
        return (cv::Mat_<double>(3, 3) << 100, 0, double(img_width) / 2,
                                          0, 100, double(img_height) / 2,
                                          0, 0, 1);
    }

    cv::Mat getDistCoeffs() {
        return (cv::Mat_<float>(5, 1) << 0, 0, 0, 0, 0);
    }

    void movePosition(float x, float y, float z) {
        position.at<double>(0) += double(x);
        position.at<double>(1) += double(y);
        position.at<double>(2) += double(z);
    }

    bool doMultiFrameInit() {
        cv::Mat current_image = polygons_space.getImage(rotation, position, getCameraMatrix(), getDistCoeffs(), cv::Size(img_width, img_height));
        return local_slam.multi_frame_init(current_image);
    }

    bool doUpdate(Pose3D& pose) {
        cv::Mat current_image = polygons_space.getImage(rotation, position, getCameraMatrix(), getDistCoeffs(), cv::Size(img_width, img_height));
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

    // TODO: check pose with more movement. Need to allow the position is scaled without extarnal pose data.
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

