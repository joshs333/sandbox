#include "jcontrols/models/bicycle_model3.h"

#include <cmath>
#include <iostream>

namespace jcontrols {
namespace models {

BicycleModel3::BicycleModel3(BicycleModel3::Params params):
    params_(params)
{}

BicycleModel3::XMatrix BicycleModel3::f(XMatrix state, UMatrix controls) {
    BicycleModel3::XMatrix res;
    res <<  state(3,0) * std::cos(state(2,0)),
            state(3,0) * std::sin(state(2,0)),
            state(3,0) * std::tan(controls(0,0)) / params_.L,
            controls(1,0);
    return res;
}

BicycleModel3::AMatrix BicycleModel3::A(XMatrix state, UMatrix controls) {

    BicycleModel3::AMatrix res;
    res <<  0., 0., 0., std::cos(state(2,0)),
            0., 0., 0., std::sin(state(2,0)),
            0., 0., 0., std::tan(controls(0,0)) / params_.L,
            0., 0., 0., 0.;
    return res;
}

BicycleModel3::BMatrix BicycleModel3::B(XMatrix state, UMatrix controls) {
    (void) state;
    (void) controls;

    BicycleModel3::BMatrix res;
    res <<  0, 0,
            0, 0,
            state(3,0) / (std::cos(controls(0,0)) * std::cos(controls(0,0)) * params_.L), 0,
            0, 1;
    return res;
}

}; /* namespace jcontrols */
}; /* namespace models */