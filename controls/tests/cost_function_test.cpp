#include "gtest/gtest.h"

#include "jcontrols/cost_functions/quadratic.h"

TEST(CostFunctionTest, QuadraticCostVQFQR) {
    typedef jcontrols::cost_functions::QuadraticCostFunction<2,1> CF;
    CF cost_function(
        {3., 4.},
        {2., 3.},
        {1.}
    );
}

TEST(CostFunctionTest, QuadraticCostVQR) {
    typedef jcontrols::cost_functions::QuadraticCostFunction<2,1> CF;
    CF cost_function(
        {2., 3.},
        {1.}
    );
}

TEST(CostFunctionTest, QuadraticCostDQFQR) {
    typedef jcontrols::cost_functions::QuadraticCostFunction<2,1> CF;
    CF cost_function(3., 2., 1.);
}

TEST(CostFunctionTest, QuadraticCostDQR) {
    typedef jcontrols::cost_functions::QuadraticCostFunction<2,1> CF;
    CF cost_function(2.0, 1.0);
}