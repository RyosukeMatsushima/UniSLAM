#ifndef VECTOR_AVERAGE_HPP
#define VECTOR_AVERAGE_HPP

#include <Eigen/Dense>
#include <vector>

class VectorAverage {
public:
    VectorAverage(int stock_size);

    void add_vector(const Eigen::Vector3f& vec);
    Eigen::Vector3f get_average() const;
    bool is_filled() const;
    Eigen::Vector3f get_variance() const;

    void clear();

private:
    int stock_size_;
    std::vector<Eigen::Vector3f> vectors_;
    Eigen::Vector3f current_sum_;
};

#endif // VECTOR_AVERAGE_HPP

