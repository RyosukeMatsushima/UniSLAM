#include <gtest/gtest.h>
#include "camera_model.hpp"

#include <iostream>

TEST(CameraModelTest, getDirectionAndAngle) {

    std::vector<cv::Point3f> points_3d = {
        cv::Point3f(1, 0, 1),
        cv::Point3f(-1, 0, 1),
        cv::Point3f(1, 1, 1),
        cv::Point3f(0, 1, 1),
        cv::Point3f(1, 2, 1),
        cv::Point3f(0, 2, 1)
    };

    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

    cv::Size image_size(1000, 1000);

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 600, 0, float(image_size.width) / 2,
                                                       0, 600, float(image_size.height) / 2,
                                                       0, 0, 1);

    CameraModel camera_model(camera_matrix, image_size);


    for (const auto &point: points_3d)
    {
        std::vector<cv::Point3f> point_3d = {point};
        std::vector<cv::Point2f> projected_points;
        cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F); // Assuming no lens distortion for now
        cv::projectPoints(point_3d, rvec, tvec, camera_matrix, dist_coeffs, projected_points);


        EdgePoint edge_point(projected_points[0], cv::Vec2f(1, 0));
        edge_point.id = 1;

        EdgeNode edge_node = camera_model.getEdgeNode(edge_point);

        Eigen::Vector3f expected_direction_frame_to_edge(point.x, point.y, point.z);
        expected_direction_frame_to_edge.normalize();

        // TODO: edge_direction is changed if camera_model supports distortion
        // direction is 90 degree rotated normalised gradient
        Eigen::Vector2f expected_edge_direction(0, 1);

        EXPECT_EQ(edge_node.direction_frame_to_edge, expected_direction_frame_to_edge);
        EXPECT_EQ(edge_node.edge_direction, expected_edge_direction);
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

