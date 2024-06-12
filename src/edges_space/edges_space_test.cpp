#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "edges_space.h"

#define RESULT_IMAGE_PATH "./result/"

TEST(EdgesSpaceTest, TestEdgesSpace) {
  EdgesSpace edges_space;

  // draw square in x-z plane
  cv::Point3f p1(-1, 1, 0);
  cv::Point3f p2(1, 1, 0);
  cv::Point3f p3(1, 1, 1);
  cv::Point3f p4(-1, 1, 1);

  cv::Point3f p5(-1, -1, 0);
  cv::Point3f p6(1, -1, 0);
  cv::Point3f p7(1, -1, 1);
  cv::Point3f p8(-1, -1, 1);

  edges_space.setEdge(p1, p2);
  edges_space.setEdge(p2, p3);
  edges_space.setEdge(p3, p4);
  edges_space.setEdge(p4, p1);

  edges_space.setEdge(p5, p6);
  edges_space.setEdge(p6, p7);
  edges_space.setEdge(p7, p8);
  edges_space.setEdge(p8, p5);

  int width = 500;
  int height = 500;
  cv::Size img_size(width, height);

  // camera parameters as CV_64F
  cv::Mat K = (cv::Mat_<double>(3, 3) << 100, 0, float(width) / 2, 0, 100, float(height) / 2, 0, 0, 1);
  cv::Mat R = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  cv::Mat t = (cv::Mat_<double>(3, 1) << 0, 0, 1);
  cv::Mat dist = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);


  cv::Mat out_put_img = edges_space.getImage(R, t, K, dist, img_size);

  std::cout << "out_put_img.channels() = " << out_put_img.channels() << std::endl;

  std::vector<cv::Mat> channels;
  cv::split(out_put_img, channels);

  for (int i = 0; i < channels.size(); i++) {
    std::string file_name = RESULT_IMAGE_PATH "edges_space_test_channel_" + std::to_string(i) + ".png";

    cv::normalize(channels[i], channels[i], 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imwrite(file_name, channels[i]);
  }
}

