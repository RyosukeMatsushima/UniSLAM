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
    EXPECT_FALSE(vector_average.is_filled());

    vector_average.add_vector(v2);
    average = vector_average.get_average();

    EXPECT_TRUE(vector_average.is_filled());

    EXPECT_EQ(average[0], 1);
    EXPECT_EQ(average[1], 0);
    EXPECT_EQ(average[2], 0);

    vector_average.add_vector(v3);
    vector_average.add_vector(v4);

    EXPECT_TRUE(vector_average.is_filled());

    average = vector_average.get_average();

    EXPECT_EQ(average[0], 1);
    EXPECT_EQ(average[1], 0);
    EXPECT_EQ(average[2], 0);
}

TEST(VectorAverageTest, variance) {

    int stock_size = 6;
    
    VectorAverage vector_average(stock_size);

    Eigen::Vector3f v1(1, -1, 0);
    Eigen::Vector3f v2(1, 1, 0);
    Eigen::Vector3f v3(1, -1, 0);
    Eigen::Vector3f v4(1, 1, -1);
    Eigen::Vector3f v5(1, 1, -1);
    Eigen::Vector3f v6(1, -1, -1);

    vector_average.add_vector(v1);
    vector_average.add_vector(v2);
    vector_average.add_vector(v3);
    vector_average.add_vector(v4);
    vector_average.add_vector(v5);
    vector_average.add_vector(v6);

    Eigen::Vector3f variance = vector_average.get_variance();

    EXPECT_EQ(variance[0], 0);
    EXPECT_EQ(variance[1], 1);
    EXPECT_EQ(variance[2], 0.25);
}

