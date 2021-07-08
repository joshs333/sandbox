#include "gtest/gtest.h"

#include "jcontrols/models/bicycle_model.h"
#include "jcontrols/models/discretized_dynamics.h"
#include "jcontrols/models/pendulum.h"

// Demonstrate some basic assertions.
TEST(DynamicsTest, DiscretizeBicyleModel) {
    typedef jcontrols::models::BicycleModel BM;
    typedef jcontrols::models::DiscretizedDynamics<BM> DD;

    BM::Params params;
    BM continuous_bm(params);
    DD discrete_bm(continuous_bm, 0.1);
}