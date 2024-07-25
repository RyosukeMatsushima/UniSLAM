#include <gtest/gtest.h>
#include "vector_average.hpp"

TEST(VectorAverageTest, average) {

    int stock_size = 2;
    VectorAverage vector_average(stock_size);

    Eigen::Vector3f v1(1, -1, 0);
    Eigen::Vector3f v2(1, 1, 0);
    Eigen::Vector3f v3(1, -1, 1);
    Eigen::Vector3f v4(1, 1, -1);

    vector_average.add_vector(v1);
    Eigen::Vector3f average = vector_average.get_average();

    EXPECT_EQ(average[0], 1);
    EXPECT_EQ(average[1], -1);
    EXPECT_EQ(average[2], 0);

    vector_average.add_vector(v2);
    average = vector_average.get_average();

    EXPECT_EQ(average[0], 1);
    EXPECT_EQ(average[1], 0);
    EXPECT_EQ(average[2], 0);

    vector_average.add_vector(v3);
    vector_average.add_vector(v4);

    average = vector_average.get_average();

    EXPECT_EQ(average[0], 1);
    EXPECT_EQ(average[1], 0);
    EXPECT_EQ(average[2], 0);
}

