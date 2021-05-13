#include "jcontrols/models/bicycle_model.h"

#include <cmath>
#include <iostream>

namespace jcontrols {
namespace models {

BicycleModel::BicycleModel(BicycleModel::Params params):
    params_(params)
{}

BicycleModel::XMatrix BicycleModel::f(XMatrix state, UMatrix controls) {
    BicycleModel::XMatrix res;
    res <<  state(3,0) * std::cos(state(2,0)),
            state(3,0) * std::sin(state(2,0)),
            state(3,0) * std::tan(controls(0,0)) / params_.L,
            controls(1,0);
    return res;
}

BicycleModel::AMatrix BicycleModel::A(XMatrix state, UMatrix controls) {

    BicycleModel::AMatrix res;
    res <<  0., 0., -state(3,0) * std::sin(state(2,0)),     std::cos(state(2,0)),
            0., 0., state(3,0) * std::cos(state(2,0)),     std::sin(state(2,0)),
            0., 0., 0., std::tan(controls(0,0)) / params_.L,
            0., 0., 0., 0.;
    return res;
}

BicycleModel::BMatrix BicycleModel::B(XMatrix state, UMatrix controls) {
    (void) state;
    (void) controls;

    BicycleModel::BMatrix res;
    res <<  0, 0,
            0, 0,
            state(3,0) / (std::cos(controls(0,0)) * std::cos(controls(0,0)) * params_.L), 0,
            0, 1;
    return res;
}

}; /* namespace jcontrols */
}; /* namespace models */