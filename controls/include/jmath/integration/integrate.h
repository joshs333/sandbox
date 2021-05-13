#ifndef JMATH_INTEGRATION_INTEGRATE_H
#define JMATH_INTEGRATION_INTEGRATE_H

#include <utility>
#include <iostream>
#include "jmath/time_range.h"

namespace jmath {
namespace integration {

template<typename IntM, int x_dim, typename scalar_T=double,
typename F = std::function<Eigen::Matrix<scalar_T, x_dim, 1>(Eigen::Matrix<scalar_T, x_dim, 1>, double)>>
std::vector<Eigen::Matrix<scalar_T, x_dim, 1>>
integrate(F f, Eigen::Matrix<scalar_T, x_dim, 1> x_0, TimeRange range) {
    std::vector<Eigen::Matrix<scalar_T, x_dim, 1>> results(range.size() + 1);
    Eigen::Matrix<scalar_T, x_dim, 1> x_n;
    x_n = x_0;

    results[0] = x_0;

    for (auto it = range.begin(); it != range.end(); ++it) {
        x_n = IntM::integrate(f, x_n, it->size, it->time);
        results[it->step + 1] = Eigen::Matrix<scalar_T, x_dim, 1>(x_n);
    }
    return results;
}

}; /* namespace integration */
}; /* namespace jmath */

#endif /* JMATH_INTEGRATION_INTEGRATE_H */