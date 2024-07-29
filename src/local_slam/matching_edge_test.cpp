#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "polygons_space.hpp"
#include "camera_model.hpp"
#include "frame_node.hpp"
#include "line_3d.hpp"
#include "debug_view.hpp"
#include "local_slam.hpp"

#define RESULT_IMAGE_PATH PROJECT_SOURCE_DIR "/test/result/"
#define CONFIG_FILE_PATH PROJECT_SOURCE_DIR "/config_for_test/"

class MatchingEdgeTest : public ::testing::Test {
protected:
    PolygonsSpace polygons_space;

    cv::Mat position = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    cv::Mat rotation = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                                                  0, 1, 0,
                                                  0, 0, 1);

    cv::Size image_size = cv::Size(1000, 1000);
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 600, 0, image_size.width / 2,
                                                      0, 600, image_size.height / 2,
                                                      0, 0, 1);
    
    CameraModel camera_model;
    LocalSlam local_slam;

    float square_size = 0.5;
    Eigen::Vector3f p1 = Eigen::Vector3f(-square_size, -square_size, 1);
    Eigen::Vector3f p2 = Eigen::Vector3f(-square_size, square_size, 1);
    Eigen::Vector3f p3 = Eigen::Vector3f(square_size, square_size, 1);
    Eigen::Vector3f p4 = Eigen::Vector3f(square_size, -square_size, 1);

    MatchingEdgeTest() :
        polygons_space(),
        camera_model(camera_matrix, image_size),
        local_slam(camera_model, CONFIG_FILE_PATH "edge_space_dynamics.yaml") {

        std::vector<cv::Point3f> polygon = {cv::Point3f(p1.x(), p1.y(), p1.z()),
                                            cv::Point3f(p2.x(), p2.y(), p2.z()),
                                            cv::Point3f(p3.x(), p3.y(), p3.z()),
                                            cv::Point3f(p4.x(), p4.y(), p4.z())};
        polygons_space.setPolygon(polygon, cv::Scalar(255, 0, 0));
    }

    std::vector<Line3D> getExpectedEdges() {
        std::vector<Line3D> edges;
        edges.push_back(Line3D(0, p1, p2 - p1, (p2 - p1).norm()));
        edges.push_back(Line3D(0, p2, p3 - p2, (p3 - p2).norm()));
        edges.push_back(Line3D(0, p3, p4 - p3, (p4 - p3).norm()));
        edges.push_back(Line3D(0, p4, p1 - p4, (p1 - p4).norm()));
        return edges;
    }

    void movePosition(float x, float y, float z) {
        position.at<double>(0) += double(x);
        position.at<double>(1) += double(y);
        position.at<double>(2) += double(z);
    }

    Eigen::Vector3f getPosition() const {
        return Eigen::Vector3f(float(position.at<double>(0)), float(position.at<double>(1)), float(position.at<double>(2)));
    }

    Eigen::Quaternionf getOrientation() const {
        Eigen::Matrix3f rotation_matrix;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                rotation_matrix(i, j) = float(rotation.at<double>(i, j));
            }
        }
        Eigen::Quaternionf q(rotation_matrix);
        return q;
    }

    // max acceptable error is 1 pixel
    float getDistanceThreshold(const EdgePoint& edge_point, const float distance_to_edge) const {
        EdgeNode edge_node = camera_model.getEdgeNode(edge_point);
        EdgePoint shifted_edge_point(edge_point.point + cv::Point2f(1, 1), edge_point.gradient);
        EdgeNode shifted_edge_node = camera_model.getEdgeNode(shifted_edge_point);

        Line3D observed_line(0, getPosition(), edge_node.direction_frame_to_edge, 0.0f);
        Line3D shifted_observed_line(0, getPosition(), shifted_edge_node.direction_frame_to_edge, 0.0f);

        return (observed_line.get_point_at(distance_to_edge) - shifted_observed_line.get_point_at(distance_to_edge)).norm();
    }

    bool checkEdgeNode(const EdgeNode& edge_node, const EdgePoint& edge_point) {

        std::cout << std::endl;
        std::cout << "checkEdgeNode" << std::endl;

        int ok_edges = 0;
        float angle_threshold = 0.1f;

        for (const auto& expected_edge : getExpectedEdges()) {
            Eigen::Vector2f expected_edge_direction = expected_edge.direction().head<2>();
            float angle = std::acos(expected_edge_direction.dot(edge_node.edge_direction));

            Line3D observed_line(0, getPosition(), edge_node.direction_frame_to_edge, 0.0f);

            float distance_to_observed_line, distance_to_expected_edge;
            Line3D::get_closest_points_between(expected_edge, observed_line, distance_to_observed_line, distance_to_expected_edge);
            float distance = (expected_edge.get_point_at(distance_to_observed_line) - observed_line.get_point_at(distance_to_expected_edge)).norm();

            float distance_threshold = getDistanceThreshold(edge_point, distance_to_expected_edge);

            std::cout << "distance: " << distance << std::endl;
            std::cout << "angle: " << angle << std::endl;
            std::cout << "distance_threshold: " << distance_threshold << std::endl;
            if (distance < distance_threshold && angle < angle_threshold) {
                ok_edges++;
            }
        }

        std::cout << "ok_edges: " << ok_edges << std::endl;
        EXPECT_EQ(ok_edges, 1) << "Invalid edge node: " << std::endl
                               << "ok_edges: " << ok_edges << std::endl;
        return ok_edges == 1;
    }
};

TEST_F(MatchingEdgeTest, matching_edge) {
    float dxy_position = 0.1f;
    int window_size = 200;
    float angle_resolution = 0.2f;

    cv::Mat image1 = polygons_space.getImage(rotation, position, camera_matrix, image_size);
    FrameNode frame_node1(image1, window_size, angle_resolution);
    std::vector<EdgePoint> edge_points = frame_node1.findNewEdgePoints();
    Pose3D pose1 = Pose3D(getPosition(), getOrientation());

    // Check edge points
    for (const auto& edge_point : edge_points) {
        EdgeNode edge_node = camera_model.getEdgeNode(edge_point);
        ASSERT_TRUE(checkEdgeNode(edge_node, edge_point)) << "invalid edge node with first frame";
    }

    movePosition(dxy_position, dxy_position, 0.0f);

    cv::Mat image2 = polygons_space.getImage(rotation, position, camera_matrix, image_size);
    FrameNode frame_node2(image2, window_size, angle_resolution);

    std::vector<EdgePoint> edge_points2 = frame_node2.findNewEdgePoints();
    Pose3D pose2 = Pose3D(getPosition(), getOrientation());

    // Check edge points in frame_node2
    for (const auto& edge_point : edge_points2) {
        EdgeNode edge_node = camera_model.getEdgeNode(edge_point);
        ASSERT_TRUE(checkEdgeNode(edge_node, edge_point)) << "invalid edge node with second frame";
    }

    // Make debug images
    DebugView debug_view(image1);
    DebugView debug_view2(image2);
    debug_view.drawEdgePoints(edge_points, cv::Scalar(0, 255, 255));
    debug_view2.drawEdgePoints(edge_points, cv::Scalar(0, 255, 255));

    for (const auto& edge_point : edge_points) {
        EdgePoint matched_edge_point;
        if (!frame_node2.matchEdge(edge_point, matched_edge_point)) continue;

        EdgeNode matched_edge_node = camera_model.getEdgeNode(matched_edge_point);

        checkEdgeNode(matched_edge_node, matched_edge_point);

        debug_view.drawEdgePoints({matched_edge_point}, cv::Scalar(255, 255, 0));
        debug_view2.drawEdgePoints({matched_edge_point}, cv::Scalar(255, 255, 0));
    }

    cv::imwrite(RESULT_IMAGE_PATH "matching_edge.png", debug_view.getDebugImage());
    cv::imwrite(RESULT_IMAGE_PATH "matching_edge2.png", debug_view2.getDebugImage());

    // check edge pose calculation
    local_slam.fix_edges(frame_node1, frame_node2, pose1, pose2);

    DebugView debug_view_calculate_edge(image1);
    debug_view_calculate_edge.drawEdgePoints(frame_node1.getFixedEdgePoints(), cv::Scalar(0, 225, 255));
    debug_view_calculate_edge.drawEdgePoints(frame_node2.getFixedEdgePoints(), cv::Scalar(225, 255, 0));
    cv::imwrite(RESULT_IMAGE_PATH "calculate_edge_from_pose1.png", debug_view_calculate_edge.getDebugImage());

    for (const auto& edge_line: local_slam.get_fixed_edges()) {
        std::cout << "" << std::endl;
        std::cout << "id: " << edge_line.id() << std::endl;
        std::cout << "start_point: " << edge_line.start_point().transpose() << std::endl;
        std::cout << "direction: " << edge_line.direction().transpose() << std::endl;
        std::cout << "length: " << edge_line.length() << std::endl;
    }
}

