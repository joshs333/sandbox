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
using jcontrols::DiscretizedDynamics2;
using namespace std::placeholders;
using namespace jmath::integration;
using namespace jmath;

Eigen::Matrix<double, 3,3> jac(
    Eigen::Matrix<double, 3, 1> x,
    double t,
    Pendulum* pp,
    Pendulum::UVector u0
) {

    auto ab = pp->taylor(x.block<2,1>(0,0), u0);
    
    Eigen::Matrix<double, 3,3> res = Eigen::Matrix<double, 3,3>::Zero();
    res.block<2,2>(0,0) = ab.first;
    res.block<2,1>(0,2) = ab.second;
    // res(2,0) = res(2,1) = res(2,2) = 0;

    return res;
}

Eigen::Matrix<double, 3,1> f(
    Eigen::Matrix<double, 3, 1> x,
    double t,
    Pendulum* pp,
    Pendulum::UVector u0
) {

    Eigen::Matrix<double, 3,1> res = Eigen::Matrix<double, 3,1>::Zero();
    res.block<2,1>(0,0) = pp->f(x.block<2,1>(0,0), x.block<1,1>(2,0));
    return res;
}

const double dt = 0.01;
int main(int argc, char** argv) {
    (void) argc;
    (void) argv;
    std::printf("Hello!\n");

    Pendulum::XVector x0;
    x0 << 0,0;
    Pendulum::UVector u0;
    u0 << 0;

    Eigen::Matrix<double,3,1> xx0;
    xx0 << x0(0,0), x0(1,0), u0(0,0);

    Pendulum::Params p;
    Pendulum pp(p);
    DiscretizedDynamics<RK4Classic, Pendulum> ppp(pp, dt);
    DiscretizedDynamics2<RK4Classic, Pendulum> ppp2(pp, dt);

    int trials = 1000;
    auto t1 = high_resolution_clock::now();
    for(int i = 0; i < trials; ++i) {
        auto r = RK4Classic::jacobian<3>(
            std::bind(&f, std::placeholders::_1, std::placeholders::_2, &pp, u0),
            std::bind(&jac, std::placeholders::_1, std::placeholders::_2, &pp, u0),
            xx0,
            dt
        );
    }
    auto t2 = high_resolution_clock::now();
    duration<double, std::milli> diff = t2 - t1;
    std::cout << "Took " << diff.count() << " ms" << std::endl;

    t1 = high_resolution_clock::now();
    for(int i = 0; i < trials; ++i) {
        auto r2 = RK4Classic::jacobian2<2,1>(
            std::bind(&Pendulum::f, &pp, std::placeholders::_1, u0),
            std::bind(&Pendulum::taylor, &pp, std::placeholders::_1, u0),
            x0,
            dt
        );
    }
    t2 = high_resolution_clock::now();
    diff = t2 - t1;
    std::cout << "Took " << diff.count() << " ms" << std::endl;

    // std::cout << "jacboian2" << std::endl;
    // std::cout << r2.first << std::endl;
    // std::cout << r2.second << std::endl;

    // std::cout << "jacboian" << std::endl;
    // std::cout << r << std::endl;

    return 0;
}