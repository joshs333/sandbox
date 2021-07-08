#ifndef JMATH_INTEGRATION_EULER_H
#define JMATH_INTEGRATION_EULER_H

#include <Eigen/Core>
#include <utility>

namespace jmath {
namespace integration {

// I have somehow become this person... oh no.
struct Euler {
    template<int x_dim, typename scalar_T=double,
    typename F = std::function<Eigen::Matrix<scalar_T, x_dim, 1>(Eigen::Matrix<scalar_T, x_dim, 1>, double)>>
    static Eigen::Matrix<scalar_T, x_dim, 1>
    integrate(
        F f,
        Eigen::Matrix<scalar_T, x_dim, 1> x_0,
        double dt,
        double t_0 = 0
    ) {
        typedef Eigen::Matrix<scalar_T, x_dim, 1> XM;
        XM k1;
        k1 = dt * f(x_0, t_0);
        return x_0 + k1;
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
        typedef Eigen::Matrix<scalar_T, x_dim, x_dim> JM;
        JM eye, A1;
        eye = JM::Identity();
        A1 = dt * dfdx(x_0, t_0);
        return eye + A1;
    }
}; /* struct Euler */


}; /* namespace integration */
}; /* namespace jmath */

#endif /* JMATH_INTEGRATION_RK4_CLASSIC_H */