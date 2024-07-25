#ifndef VECTOR_AVERAGE_HPP
#define VECTOR_AVERAGE_HPP

#include <Eigen/Dense>
#include <vector>

class VectorAverage {
public:
    VectorAverage(int stock_size);

    void add_vector(const Eigen::Vector3f& vec);
    Eigen::Vector3f get_average() const;

private:
    int stock_size_;
    std::vector<Eigen::Vector3f> vectors_;
    Eigen::Vector3f current_sum_;
};

#endif // VECTOR_AVERAGE_HPP

