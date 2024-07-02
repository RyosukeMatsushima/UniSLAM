#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "polygons_space.hpp"

#ifndef PROJECT_SOURCE_DIR
#error "PROJECT_SOURCE_DIR is not defined"
#endif

#define RESULT_IMAGE_PATH PROJECT_SOURCE_DIR "/test/result/"

TEST(PolygonsSpaceTest, TestPolygonsSpace) {
  PolygonsSpace polygons_space;

  // draw square in x-z plane
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

  int width = 500;
  int height = 500;
  cv::Size img_size(width, height);

  // camera parameters as CV_64F
  cv::Mat K = (cv::Mat_<double>(3, 3) << 100, 0, float(width) / 2, 0, 100, float(height) / 2, 0, 0, 1);
  cv::Mat R = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  cv::Mat t_1 = (cv::Mat_<double>(3, 1) << 0, 0, 1);
  cv::Mat t_2 = (cv::Mat_<double>(3, 1) << 0.5, 0, 1);

  std::vector<cv::Mat> t = {t_1, t_2};

  for (auto &tvec: t)
  {
    cv::Mat out_put_img = polygons_space.getImage(R, tvec, K, img_size);

    std::string file_name = RESULT_IMAGE_PATH "polygons_space_test_tvec_" + std::to_string(tvec.at<double>(0)) + ".png";
    cv::imwrite(file_name, out_put_img);
  }
}

