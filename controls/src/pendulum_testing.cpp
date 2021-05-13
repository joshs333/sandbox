#include <cstdio>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "jcontrols/dynamics.h"
#include "jcontrols/models/pendulum.h"
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

// typedef jmath::integration::Euler CIntM;
// typedef jmath::integration::RK4Classic DIntM;
typedef jmath::integration::Euler DIntM;
typedef jcontrols::models::Pendulum CPD;
typedef jcontrols::DiscretizedDynamics<CPD, DIntM> DPD;
typedef jcontrols::QuadraticCostFunction<DPD::state_dim,DPD::control_dim> CF;
// https://bjack205.github.io/papers/AL_iLQR_Tutorial.pdf
// https://arxiv.org/pdf/2007.14492.pdf
// https://homes.cs.washington.edu/~todorov/papers/LiICINCO04.pdf
// http://roboticexplorationlab.org/papers/iLQR_Tutorial.pdf


const double dt = 0.02;
const int horizon = 50;

int main(int argc, char** argv) {
    (void) argc;
    (void) argv;


    DPD::UMatrix zero_control(6.);
    std::vector<DPD::UMatrix> utraj(horizon);
    for(int i = 0; i < horizon; ++i) {
        utraj[i] << zero_control;
    }

    DPD::XMatrix start;
    start << -M_PI / 2, 0;
    DPD::XMatrix goal;
    goal << 0, 0;

    CF cost_function = jcontrols::QuickQuadraticCost<DPD::state_dim,DPD::control_dim>(
        { 400, 10 },
        { 2e-3 },
        { 10000,10000 }
    );

    CPD::Params pendulum_params;
    CPD continuous_pendulum(pendulum_params);
    DPD discretized_pendulum(continuous_pendulum, dt);
    jcontrols::ilqr::TrackGoalProblem<DPD, CF> problem(&discretized_pendulum, &cost_function, horizon, start, goal);

    ilqr::Solver ilqr_solver;
    ilqr_solver.params().epsilon(1e-3);

    int trials = 1;
    auto t1 = high_resolution_clock::now();
    for(int i = 0; i < trials; ++i) {
        ilqr_solver.solve(problem, utraj);
    }
    auto t2 = high_resolution_clock::now();
    duration<double, std::milli> diff = t2 - t1;
    std::cout << "Took " << diff.count() << " ms" << std::endl;


    return 0;
}