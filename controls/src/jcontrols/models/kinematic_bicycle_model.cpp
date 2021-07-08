#include "jcontrols/models/kinematic_bicycle_model.h"

#include <cmath>
#include <iostream>

#include "jmath/utils.h"

namespace jcontrols {
namespace models {

KinematicBicycleModel::KinematicBicycleModel(Params params):
    params_(params)
{}

KinematicBicycleModel::XVector KinematicBicycleModel::f(XVector state, UVector control) {
    KinematicBicycleModel::XVector res;
    res <<  state(3,0) * std::cos(state(2,0)),
            state(3,0) * std::sin(state(2,0)),
            state(3,0) * std::tan(control(0,0)) / params_.L,
            control(1,0);
    return res;
}


KinematicBicycleModel::TaylorExpansion
KinematicBicycleModel::taylor(XVector state, UVector control) {
    KinematicBicycleModel::TaylorExpansion res;
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


void KinematicBicycleModel::wrap(XVector& state) {
    state(2,0) = jmath::wrap_angle(state(2,0));
}

}; /* namespace jcontrols */
}; /* namespace models */