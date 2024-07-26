#include "vector_average.hpp"

VectorAverage::VectorAverage(int stock_size) 
    : stock_size_(stock_size), current_sum_(Eigen::Vector3f::Zero()) {}

void VectorAverage::add_vector(const Eigen::Vector3f& vec) {
    if (vectors_.size() == stock_size_) {
        current_sum_ -= vectors_.front();
        vectors_.erase(vectors_.begin());
    }

    vectors_.push_back(vec);
    current_sum_ += vec;
}

Eigen::Vector3f VectorAverage::get_average() const {
    if (vectors_.empty()) {
        return Eigen::Vector3f::Zero();
    }

    return current_sum_ / static_cast<float>(vectors_.size());
}

bool VectorAverage::is_filled() const {
    return vectors_.size() == stock_size_;
}

Eigen::Vector3f VectorAverage::get_variance() const {
    if (vectors_.empty()) {
        return Eigen::Vector3f::Zero();
    }

    Eigen::Vector3f average = get_average();
    Eigen::Vector3f variance = Eigen::Vector3f::Zero();

    for (const Eigen::Vector3f& vec : vectors_) {
        variance += (vec - average).cwiseAbs2();
    }

    return variance / static_cast<float>(vectors_.size());
}
