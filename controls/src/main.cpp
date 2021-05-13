#include <cstdio>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "matplotlib.h"
#include "jcontrols/dynamics.h"
// #include "jcontrols/models/pendulum.h"
#include "jcontrols/models/bicycle_model.h"
#include "jcontrols/models/bicycle_model2.h"
#include "jcontrols/models/bicycle_model3.h"
#include "jmath/integration/integrate.h"
#include "jmath/integration/euler.h"
#include "jmath/integration/rk4_classic.h"
#include "jcontrols/cost_function.h"
#include "jmath/time_range.h"
#include "jcontrols/ilqr/problem.h"
#include "jcontrols/ilqr/solver.h"
#include <iostream>
#include <chrono>
#include <vector>
#include <array>

using namespace jcontrols;
using namespace jcontrols::models;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;
namespace plt = matplotlibcpp;

// typedef jmath::integration::Euler CIntM;
// typedef jmath::integration::RK4Classic DIntM;
typedef jmath::integration::Euler DIntM;
typedef jcontrols::models::BicycleModel CPD;
typedef jcontrols::DiscretizedDynamics<CPD, DIntM> DPD;
typedef jcontrols::QuadraticCostFunction<DPD::state_dim,DPD::control_dim> CF;
typedef jcontrols::ilqr::TrackTrajProblem<DPD, CF> Problem;
// https://bjack205.github.io/papers/AL_iLQR_Tutorial.pdf
// https://arxiv.org/pdf/2007.14492.pdf
// https://homes.cs.washington.edu/~todorov/papers/LiICINCO04.pdf
// http://roboticexplorationlab.org/papers/iLQR_Tutorial.pdf


const double dt = 0.01;
const int horizon = 50;
const int track_len = 1000;
const int finer_dt = 10;

void sampleTraj(DPD::XMatrix xn, std::vector<DPD::XMatrix>& xm, std::vector<DPD::UMatrix> um, DPD d) {
    DPD::UMatrix un(0,0);
    // A bit of manual tuning for this number
    // really I just wanted the path to look pretty on the plot
    jmath::TimeRange r(0., track_len, dt);
    double un_dir = 0.005;
    double un_max = 0.5;
    double v_max = 2.0;
    xm.push_back(xn);
    for(auto it = r.begin(); it != r.end(); ++it) {
        if(un(0,0) > un_max && un_dir > 0.) {
            un_dir *= -1;
        }
        if(un(0,0) < -un_max && un_dir < 0.) {
            un_dir *= -1;
        }
        un(0,0) += un_dir;
        
        if(xn(3,0) < v_max) {
            un(1,0) = 1.0;
        } else {
            un(1,0) = 0.0;
        }
        xn = d.f(xn, un);
        um.push_back(un);
        xm.push_back(xn);
    }
}

void plotTraj(std::vector<DPD::XMatrix>& xm, std::vector<double>& xs, std::vector<double>& ys) {
    xs.clear();
    ys.clear();
    for(int i = 0; i < xm.size(); ++i) {
        xs.push_back(xm[i](0,0));
        ys.push_back(xm[i](1,0));
    }
}

double ptDist(DPD::XMatrix pt, DPD::XMatrix ptb) {
    return std::sqrt(
        std::pow(pt(0,0) - ptb(0,0),2)
        + std::pow(pt(1,0) - ptb(1,0),2));
}

int getClosestPoint(std::vector<DPD::XMatrix>& xm, DPD::XMatrix pt) {
    int r = 0;
    double d = -1;
    for(int i = 0; i < xm.size(); ++i) {
        double dn = ptDist(xm[i], pt);
        if(d < 0 || dn < d) {
            r = i;
            d = dn;
        }
    }
    return r;
}

int main(int argc, char** argv) {
    (void) argc;
    (void) argv;



    DPD::UMatrix zero_control(0.0, 0.0);
    std::vector<DPD::UMatrix> utraj(horizon);
    for(int i = 0; i < horizon; ++i) {
        utraj[i] << zero_control;
    }

    DPD::XMatrix start;
    start << 0, 0, 0, 0;
    DPD::XMatrix goal;
    goal << -10., 100., 0., 0;

    CF cost_function = jcontrols::QuickQuadraticCost<DPD::state_dim,DPD::control_dim>(
        // { 10, 10, 5, 1 },
        // { 1e-3, 1e-4 },
        // { 10, 10, 5, 1 }
        { 1, 1, 1, 1 },
        { 0, 1e-3 },
        { 1, 1, 1, 1 }
    );

    CPD::Params pendulum_params;
    CPD continuous_pendulum(pendulum_params);
    DPD discretized_pendulum(continuous_pendulum, dt);

    CPD::Params pendulum_params2;
    pendulum_params2.L = 1.0;
    CPD continuous_pendulum2(pendulum_params2);
    DPD discretized_pendulum2(continuous_pendulum2, dt / (float)finer_dt);


    DPD::XMatrix xn(0,0,-64 * M_PI / 385,2.0);
    std::vector<DPD::XMatrix> xm;
    std::vector<DPD::UMatrix> um;
    sampleTraj(xn, xm, um, discretized_pendulum);
    std::vector<double> xxx;
    std::vector<double> yyy;
    plotTraj(xm, xxx, yyy);
    // plt::plot(xxx, yyy);
    // plt::show();
    // return 1;

    ilqr::Solver ilqr_solver;
    ilqr_solver.params().epsilon(1e-3).max_iters(5000);

    std::vector<DPD::XMatrix> xtraj;
    std::vector<DPD::XMatrix> ttraj;
    ttraj.push_back(xn);


    int trials = 1;
    auto t1 = high_resolution_clock::now();
    for(int i = 0; i + horizon + 1 < track_len ; ++i) {
    // for(int i = 0; i < trials; ++i) {
        std::vector<DPD::XMatrix> traj(xm.begin() + i, xm.begin() + horizon + 1 + i);
        Problem problem(&discretized_pendulum, &cost_function, horizon, xn, traj);
        ilqr_solver.solve(problem, utraj, xtraj);
        // std::cout << utraj[1] << std::endl;
        for(int j = 0; j < finer_dt; ++j) {
            xn = discretized_pendulum2.f(xn, utraj[0]);
            ttraj.push_back(xn);
        }
    }

    auto t2 = high_resolution_clock::now();
    duration<double, std::milli> diff = t2 - t1;
    std::cout << "Took " << diff.count() << " ms" << std::endl;

    double crosstrak = 0.;
    for(auto it = ttraj.begin(); it != ttraj.end(); ++it) {
        int cp = getClosestPoint(xm, *it);
        crosstrak += ptDist(xm[cp], *it);
    }
    std::cout << crosstrak / (float)ttraj.size() << std::endl;

    std::cout << ttraj[ttraj.size() - 1] << std::endl;
    std::vector<double> xxx2;
    std::vector<double> yyy2;
    plotTraj(ttraj, xxx2, yyy2);
    // plotTraj(xtraj, xxx2, yyy2);

    // std::cout << xtraj[horizon] << std::endl;
    // std::cout << traj[horizon] << std::endl;

    if(! std::isnan(ttraj[horizon](0,0))) {
        plt::plot(xxx, yyy);
        plt::plot(xxx2, yyy2);
        plt::show();
    } else {
        std::cout << "Nans!!! :(" << std::endl;
    }


    return 0;
}