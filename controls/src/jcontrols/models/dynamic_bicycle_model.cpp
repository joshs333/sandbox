#include "jcontrols/models/dynamic_bicycle_model.h"

#include <cmath>
#include <iostream>

namespace jcontrols {
namespace models {

DynamicBicycleModel::DynamicBicycleModel(Params params):
    params_(params)
{}

DynamicBicycleModel::XVector DynamicBicycleModel::f(XVector state, UVector control) {
    DynamicBicycleModel::XVector res;
    res <<  state(3,0) * std::cos(state(2,0)),
            state(3,0) * std::sin(state(2,0)),
            state(3,0) * std::tan(control(0,0)) / params_.L,
            control(1,0);
    return res;
}


DynamicBicycleModel::TaylorExpansion
DynamicBicycleModel::taylor(XVector state, UVector control) {
    DynamicBicycleModel::TaylorExpansion res;
    res.first <<    0., 0., -state(3,0) * std::sin(state(2,0)),     std::cos(state(2,0)),
                    0., 0., state(3,0) * std::cos(state(2,0)),     std::sin(state(2,0)),
                    0., 0., 0., std::tan(control(0,0)) / params_.L,
                    0., 0., 0., 0.;
    res.second <<   0, 0,
                    0, 0,
                    state(3,0) / (std::cos(control(0,0)) * std::cos(control(0,0)) * params_.L), 0,
                    0, 1;
    return res;
}

}; /* namespace jcontrols */
}; /* namespace models */