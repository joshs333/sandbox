#ifndef JMATH_UTILS_H
#define JMATH_UTILS_H

#include <cmath>
#include <functional>
#include <limits>
#include <Eigen/Core>

namespace jmath {

const double infinity = std::numeric_limits<double>::infinity();

/**
 * @brief ensures an angle is in the range [-M_PI, M_PI]
 **/
template<typename T>
T wrap_angle(T angle) {
    while(angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while(angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

/**
 * @brief computes a jacobian via finite differences
 **/
template<int out_dim, int in_dim, typename scalar_T=double,
typename F = std::function<Eigen::Matrix<scalar_T, out_dim, 1>(Eigen::Matrix<scalar_T, in_dim, 1>)>>
Eigen::Matrix<scalar_T, out_dim, in_dim>
finite_difference(
    F f,
    Eigen::Matrix<scalar_T, in_dim, 1> x,
    float h = 1e-4
) {
    Eigen::Matrix<scalar_T, out_dim, in_dim> res;
    for(int i = 0; i < in_dim; ++i) {
        // Compute the results given a small change +/- h in a single input row
        auto x_diff = x;
        x_diff(i,0) = x(i,0) + h;
        auto plus_result = f(x_diff);
        x_diff(i,0) = x(i,0) - h;
        auto minus_result = f(x_diff);

        // Perform finite differences for each output row
        for(int j = 0; j < out_dim; ++j) {
            res(j,i) = (plus_result(j,0) - minus_result(j,0)) / (2. * h);
        }
    }
    return res;
}

}; /* namespace jmath */

#endif /* JMATH_UTILS_H */