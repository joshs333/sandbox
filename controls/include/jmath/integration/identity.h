#ifndef JMATH_INTEGRATION_IDENTITY_H
#define JMATH_INTEGRATION_IDENTITY_H

#include <Eigen/Core>
#include <utility>

namespace jmath {
namespace integration {

// Identity integration assumes the function is properly discretized for this dt
// and does not perform any really integration but simply steps the function

// TODO: Is there a better way to do this?
struct Identity {
    template<int x_dim, typename scalar_T=double,
    typename F = std::function<Eigen::Matrix<scalar_T, x_dim, 1>(Eigen::Matrix<scalar_T, x_dim, 1>, double)>>
    static Eigen::Matrix<scalar_T, x_dim, 1>
    integrate(
        F f,
        Eigen::Matrix<scalar_T, x_dim, 1> x_0,
        double dt,
        double t_0 = 0
    ) {
        (void) dt;
        return f(x_0, t_0);
    }

    template<int x_dim, typename scalar_T=double,
    typename F = std::function<Eigen::Matrix<scalar_T, x_dim, 1>(Eigen::Matrix<scalar_T, x_dim, 1>, double)>,
    typename DF = std::function<Eigen::Matrix<scalar_T, x_dim, x_dim>(Eigen::Matrix<scalar_T, x_dim, 1>, double)>>
    static Eigen::Matrix<scalar_T, x_dim, x_dim>
    jacobian(
        F f,
        DF dfdx,
        Eigen::Matrix<scalar_T, x_dim, 1> x_0,
        double dt,
        double t_0 = 0
    ) {
        (void) f;
        (void) dt;
        return dfdx(x_0, t_0);
    }
}; /* struct Identity */


}; /* namespace integration */
}; /* namespace jmath */

#endif /* JMATH_INTEGRATION_RK4_CLASSIC_H */