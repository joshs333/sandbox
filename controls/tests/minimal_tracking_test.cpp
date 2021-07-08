
#include "gtest/gtest.h"


#include "jcontrols/dynamics.h"
#include "jcontrols/models/discretized_dynamics.h"
#include "jcontrols/models/bicycle_model.h"

#include "jcontrols/cost_function.h"
#include "jcontrols/cost_functions/quadratic.h"

#include "jcontrols/ilqr/problem.h"
#include "jcontrols/ilqr/discrete_solver.h"


using namespace jcontrols;
using namespace jcontrols::models;
using namespace jcontrols::cost_functions;

typedef DiscretizedDynamics<BicycleModel> DiscretizedBicycleModel;
typedef QuadraticCostFunction<BicycleModel::state_dim, BicycleModel::control_dim, BicycleModel::scalar_type> QuadraticCF;
typedef ilqr::TrackTrajProblem<DiscretizedBicycleModel, QuadraticCF> BicycleModelTrackingProblem;

// Demonstrate some basic assertions.
TEST(DynamicsTest, DiscretizeBicyleModel) {
    unsigned int horizon = 10;
    double dt = 0.1;
    BicycleModel::Params params;
    BicycleModel bm(params);
    DiscretizedBicycleModel disc_bm(bm, dt);
    BicycleModelTrackingProblem::XTraj target_trajectory;
    BicycleModelTrackingProblem::XTraj tracking_trajectory;
    BicycleModelTrackingProblem::UTraj tracking_controls;
    QuadraticCF cost_function(2.0, 1.0);


    BicycleModel::XVector x_start(0,0,0,0);
    for(unsigned int i = 0; i < horizon; ++i) {
        target_trajectory.push_back(x_start);
    }

    BicycleModelTrackingProblem tracking_problem(&disc_bm, &cost_function, horizon, x_start, target_trajectory);
    ilqr::DiscreteSolver solver;
    solver.solve(tracking_problem, tracking_controls, tracking_trajectory);
}
