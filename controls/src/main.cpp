// std
#include <cstdio>
#include <iostream>
#include <chrono>
#include <vector>
#include <array>
#include <limits>
// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// JControls includes
#include "jcontrols/dynamics.h"
#include "jcontrols/models/discretized_dynamics.h"
#include "jcontrols/models/kinematic_bicycle_model.h"

#include "jcontrols/cost_function.h"
#include "jcontrols/cost_functions/quadratic.h"

#include "jcontrols/ilqr/problem.h"
#include "jcontrols/ilqr/discrete_solver.h"

#include "jtraj/plotting.h"

#include "jmath/time_range.h"
#include "jmath/utils.h"

#include "jtraj/trajectory.h"



using namespace jcontrols;
using namespace jcontrols::models;
using namespace jcontrols::cost_functions;
// using std::chrono::high_resolution_clock;
// using std::chrono::duration_cast;
// using std::chrono::duration;
// using std::chrono::milliseconds;
// namespace plt = matplotlibcpp;

typedef KinematicBicycleModel KBM;
typedef DiscretizedDynamics<KBM> DiscretizedKBM;
typedef QuadraticCostFunction<KBM::state_dim, KBM::control_dim, KBM::scalar_type> QuadraticCF;
typedef ilqr::TrackTrajProblem<DiscretizedKBM, QuadraticCF> BicycleModelTrackingProblem;


const double dt = 0.1;
const unsigned int horizon = 50;
int main(int argc, char** argv) {
    (void) argc;
    (void) argv;

    KBM::Params params;
    KBM bm(params);
    DiscretizedKBM disc_bm(bm, dt);
    BicycleModelTrackingProblem::XTraj target_trajectory;
    BicycleModelTrackingProblem::XTraj tracking_trajectory;
    BicycleModelTrackingProblem::UTraj tracking_controls;
    QuadraticCF cost_function({10., 10., 10., 10.}, {1e-3, 1e-9});

    KBM::UVector seed_traj_u(-0.5, 0.1); // control to generate the tracking controls
    KBM::UVector default_u(0., 0.);   // default u to seed the solver with
    KBM::XVector x_start(0,0,M_PI - 0.2,0);    // starting position
    KBM::XVector x_n(x_start);        // used to generate the target trajectory

    for(unsigned int i = 0; i < horizon; ++i) {
        target_trajectory.push_back(x_n);
        x_n = disc_bm.f(x_n, seed_traj_u);
        tracking_controls.push_back(default_u);
    }

    BicycleModelTrackingProblem tracking_problem(&disc_bm, &cost_function, horizon, x_start, target_trajectory);
    tracking_problem.setULimits(KBM::UVector(0.57, 1.0));
    ilqr::DiscreteSolver solver;
    solver.solve(tracking_problem, tracking_controls, tracking_trajectory);

    jtraj::plotting::plotXY(target_trajectory, false);
    jtraj::plotting::plotXY(tracking_trajectory);

    return 0;
}