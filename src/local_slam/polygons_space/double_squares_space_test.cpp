#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "double_squares_space.hpp"

#ifndef PROJECT_SOURCE_DIR
#error "PROJECT_SOURCE_DIR is not defined"
#endif

#define RESULT_IMAGE_PATH PROJECT_SOURCE_DIR "/test/result/"

TEST(DoubleSquaresSpaceTest, getImage)
{
  DoubleSquaresSpace double_squares_space;

  cv::Mat R = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                                         0, 1, 0,
                                         0, 0, 1);

  cv::Mat t_1 = (cv::Mat_<double>(3, 1) << 0, 0, 0);
  cv::Mat t_2 = (cv::Mat_<double>(3, 1) << 0.5, 0, 0);

  std::vector<cv::Mat> t = {t_1, t_2};

  for (auto &tvec: t)
  {
    cv::Mat out_put_img = double_squares_space.getImage(R, tvec);

    std::string file_name = RESULT_IMAGE_PATH "double_squares_space_test_tvec_" + std::to_string(tvec.at<double>(0)) + ".png";

    cv::imwrite(file_name, out_put_img);
  }
}

