#include <cstdio>
#include <Eigen/Core>
#include <iostream>
#include <chrono>
#include "jmath/utils.h"
#include "jmath/integration/rk4_classic.h"
#include "jmath/integration/euler.h"
#include "jcontrols/dynamics.h"
#include "jcontrols/models/pendulum.h"
#include "jmath/time_range.h"

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;
using jcontrols::models::Pendulum;
using jcontrols::DiscretizedDynamics;
using namespace std::placeholders;
using namespace jmath::integration;
using namespace jmath;

Eigen::Matrix<double, 2, 1> test(Eigen::Matrix<double, 2, 1> n, double t) {
    (void) t;
    (void) n;
    Eigen::Matrix<double, 2, 1> r;
    r << 1,1;
    return r;
}


template<int x_dim, typename scalar_T=double>
static Eigen::Matrix<scalar_T, x_dim, 1>
tintegrate(
    std::function<Eigen::Matrix<scalar_T, x_dim, 1>(Eigen::Matrix<scalar_T, x_dim, 1>, double)> f,
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

typedef Euler IntM;
// typedef Euler IntM;

int main(int argc, char** argv) {
    (void) argc;
    (void) argv;
    std::printf("Hello!\n");

    jmath::TimeRange ran(0.05, 0.05 + 10 * 0.1, 0.1);
    for(auto it = ran.begin(); it != ran.end(); ++it) {
        std::printf("%d, %3.2f. %3.2f\n", it->step, it->time, it->size);
    }

    return 0;

    Pendulum::Params pp;
    Pendulum p(pp);
    Pendulum::UMatrix u(1.);

    double dt = 0.01;
    Pendulum::XMatrix r;
    r << M_PI/2.,1.0;
    // auto r2 = rk4_classic<2, double>(std::bind(&Pendulum::f, p, _1, u), r, dt);
    // auto r2 = rk4_classic<3, double>(&test, r, 1.);

    // auto r2 = IntM::integrate(std::bind(&Pendulum::f, p, _1, u), r, dt);

    auto pd = DiscretizedDynamics<Pendulum, IntM>(p, dt);


    std::function<Eigen::Matrix<double, 2, 1>(Eigen::Matrix<double, 2, 1>, double)> pf = std::bind(&Pendulum::f, p, _1, u);
    auto rki = std::bind(&IntM::integrate<2,double>, pf, _1, dt, 0);
    std::function<Eigen::Matrix<double, 2, 1>(Eigen::Matrix<double, 1, 1>)> um = [p, dt, r](Eigen::Matrix<double, 1, 1> u) {
        std::function<Eigen::Matrix<double, 2, 1>(Eigen::Matrix<double, 2, 1>, double)> pfb = std::bind(&Pendulum::f, p, _1, u);
        auto rki = std::bind(&IntM::integrate<2,double>, pfb, _1, dt, 0);
        return rki(r);
    };

    int trials = 100000;
    auto t1 = high_resolution_clock::now();
    for(int i = 0; i < trials; ++i) {
        auto Afd = finite_difference<2, 2,double>(rki, r);
        auto Bfd = finite_difference<2, 1,double>(um, u);
    }
    auto t2 = high_resolution_clock::now();
    duration<double, std::milli> diff = t2 - t1;
    std::cout << "Took " << diff.count() << " ms" << std::endl;
    t1 = high_resolution_clock::now();
    for(int i = 0; i < trials; ++i) {
        auto A = pd.A(r, u);
        auto B = pd.B(r, u);
        // auto ab = IntM::jacobian<2,1>(std::bind(&Pendulum::f, p, _1, u), std::bind(&Pendulum::A, p, _1, u),std::bind(&Pendulum::B, p, _1, u), r, dt, 0);
    }
    t2 = high_resolution_clock::now();
    diff = t2 - t1;
    std::cout << "Took " << diff.count() << " ms" << std::endl;

    // std::cout << r2 << std::endl;
    // std::cout << dt * p.f(r, u) << std::endl;

    // std::cout << "AA: " << std::endl << ab.first << std::endl;
    // std::cout << "AFD: " << std::endl << Afd << std::endl;
    // std::cout << "BA: " << std::endl <<  ab.second << std::endl;
    // std::cout << "BFD: " << std::endl <<  Bfd << std::endl;

    return 0;
}