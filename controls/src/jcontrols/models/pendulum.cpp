#include "jcontrols/models/pendulum.h"

#include <iostream>

namespace jcontrols {
namespace models {

Pendulum::Pendulum(Pendulum::Params params):
    params_(params)
{}

Pendulum::XVector Pendulum::f(XVector state, UVector controls) {
    Pendulum::XVector res;
    res << state(1,0),
        params_.gravity * std::sin(state(0,0)) / params_.length
            + controls(0,0) / (params_.mass * params_.length * params_.length)
            - params_.friction * state(1,0);
    return res;
}

Pendulum::StepLin Pendulum::taylor(XVector state, UVector controls) {
    (void) controls;

    Pendulum::StepLin res;
    res.first << 0., 1.,
           params_.gravity * std::cos(state(0,0)) / params_.length, -params_.friction;

    res.second << 0, 1. / (params_.mass * params_.length * params_.length);
    return res;
}


}; /* namespace jcontrols */
}; /* namespace models */