#ifndef JMATH_INTEGRATION_RK4_CLASSIC_H
#define JMATH_INTEGRATION_RK4_CLASSIC_H

#include <utility>

namespace jmath {
namespace integration {

// I have somehow become this person... oh no.
struct RK4Classic {
    template<int x_dim, typename scalar_T=double,
    typename F = std::function<Eigen::Matrix<scalar_T, x_dim, 1>(Eigen::Matrix<scalar_T, x_dim, 1>, double)>>
    static Eigen::Matrix<scalar_T, x_dim, 1>
    integrate(
        F f,
        Eigen::Matrix<scalar_T, x_dim, 1> x_0,
        double dt,
        double t_0 = 0
    ) {
        auto k1 = dt * f(x_0, t_0);
        auto k2 = dt * f(x_0 + k1 / 2., t_0 + dt / 2.);
        auto k3 = dt * f(x_0 + k2 / 2., t_0 + dt / 2.);
        auto k4 = dt * f(x_0 + k3, t_0 + dt);
        return x_0 + (k1 + 2. * k2 + 2. * k3 + k4) / 6.;
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
        auto eye = Eigen::Matrix<scalar_T, x_dim, x_dim>::Identity();
        auto k1 = dt * f(x_0, t_0);
        auto k2 = dt * f(x_0 + k1 / 2., t_0 + dt / 2.);
        auto k3 = dt * f(x_0 + k2 / 2., t_0 + dt / 2.);
        // auto k4 = dt * f(x_0 + k3, t_0 + dt);
        auto A1 = dt * dfdx(x_0, t_0);
        auto A2 = dt * dfdx(x_0 + k1 / 2., t_0 + dt / 2.) * (eye + A1 / 2.);
        auto A3 = dt * dfdx(x_0 + k2 / 2., t_0 + dt / 2.) * (eye + A2 / 2.);
        auto A4 = dt * dfdx(x_0 + k3, t_0 + dt) * (eye + A3);
        return eye + (A1 + 2.0 * A2 + 2.0 * A3 + A4) / 6.0;
    }

    template<int x_dim, int u_dim, typename scalar_T=double,
    typename F = std::function<Eigen::Matrix<scalar_T, x_dim, 1>(Eigen::Matrix<scalar_T, x_dim, 1>, double)>,
    typename DFx = std::function<Eigen::Matrix<scalar_T, x_dim, x_dim>(Eigen::Matrix<scalar_T, x_dim, 1>, double)>,
    typename DFu = std::function<Eigen::Matrix<scalar_T, x_dim, u_dim>(Eigen::Matrix<scalar_T, x_dim, 1>, double)>>
    static std::pair<Eigen::Matrix<scalar_T, x_dim, x_dim>,Eigen::Matrix<scalar_T, x_dim, u_dim>>
    jacobian(
        F f,
        DFx dfdx,
        DFu dfdu,
        Eigen::Matrix<scalar_T, x_dim, 1> x_0,
        double dt,
        double t_0 = 0
    ) {

        auto eye = Eigen::Matrix<scalar_T, x_dim, x_dim>::Identity();
        auto k1 = dt * f(x_0, t_0);
        auto k2 = dt * f(x_0 + k1 / 2., t_0 + dt / 2.);
        auto k3 = dt * f(x_0 + k2 / 2., t_0 + dt / 2.);
        // auto k4 = dt * f(x_0 + k3, t_0 + dt);
        auto A1 = dt * dfdx(x_0, t_0);
        auto A2 = dt * dfdx(x_0 + k1 / 2., t_0 + dt / 2.) * (eye + A1 / 2.);
        auto A3 = dt * dfdx(x_0 + k2 / 2., t_0 + dt / 2.) * (eye + A2 / 2.);
        auto A4 = dt * dfdx(x_0 + k3, t_0 + dt) * (eye + A3);

        // TODO: verify these jacobians...
        auto B1 = dt * dfdu(x_0, t_0);
        auto B2 = dt * (dfdx(x_0 + k1 / 2., t_0 + dt / 2.) * B1 * 0.5 + dfdu(x_0 + k1 / 2., t_0 + dt / 2.));
        auto B3 = dt * (dfdx(x_0 + k2 / 2., t_0 + dt / 2.) * B2 * 0.5 + dfdu(x_0 + k2 / 2., t_0 + dt / 2.));
        auto B4 = dt * (dfdx(x_0 + k3, t_0 + dt) * B3 + dfdu(x_0 + k3, t_0 + dt));
        return std::make_pair(
            eye + (A1 + 2.0 * A2 + 2.0 * A3 + A4) / 6.0,
            (B1 + 2.0 * B2 + 2.0 * B3 + B4) / 6.0);
    }
}; /* struct RK4Classic */


}; /* namespace integration */
}; /* namespace jmath */

#endif /* JMATH_INTEGRATION_RK4_CLASSIC_H */