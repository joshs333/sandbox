#include "jcontrols/models/bicycle_model2.h"

#include <cmath>
#include <iostream>

namespace jcontrols {
namespace models {

BicycleModel2::BicycleModel2(BicycleModel2::Params params):
    params_(params)
{}

BicycleModel2::XMatrix BicycleModel2::f(XMatrix state, UMatrix controls) {
    BicycleModel2::XMatrix res;
    res <<  state(4,0) * std::cos(state(2,0)),
            state(4,0) * std::sin(state(2,0)),
            state(4,0) * std::tan(state(3,0)) / params_.L,
            controls(0,0),
            controls(1,0);
    return res;
}

BicycleModel2::AMatrix BicycleModel2::A(XMatrix state, UMatrix controls) {

    BicycleModel2::AMatrix res;
    res <<  0., 0., -state(4,0) * std::sin(state(2,0)),0.,     std::cos(state(2,0)),
            0., 0., state(4,0) * std::cos(state(2,0)),0.,     std::sin(state(2,0)),
            0., 0., state(4,0) / (std::cos(state(3,0)) * std::cos(state(3,0)) * params_.L), 0., std::tan(controls(0,0)) / params_.L,
            0., 0., 0., 0., 0.,
            0., 0., 0., 0., 0.;
    return res;
}

BicycleModel2::BMatrix BicycleModel2::B(XMatrix state, UMatrix controls) {
    (void) state;
    (void) controls;

    BicycleModel2::BMatrix res;
    res <<  0, 0,
            0, 0,
            0, 0,
            1, 0,
            0, 1;
    return res;
}

}; /* namespace jcontrols */
}; /* namespace models */