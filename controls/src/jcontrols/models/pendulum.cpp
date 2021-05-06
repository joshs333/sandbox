#include "jcontrols/models/pendulum.h"

#include <iostream>

namespace jcontrols {
namespace models {

Pendulum::Pendulum(Pendulum::Params params):
    params_(params)
{}

Pendulum::XMatrix Pendulum::f(XMatrix state, UMatrix controls) {
    Pendulum::XMatrix res;
    res << state(1,0),
        params_.gravity * std::sin(state(0,0)) / params_.length
            + controls(0,0) / (params_.mass * params_.length * params_.length)
            - params_.friction * state(1,0);
    return res;
}

Pendulum::AMatrix Pendulum::A(XMatrix state, UMatrix controls) {
    (void) controls;

    Pendulum::AMatrix res;
    res << 0., 1.,
           params_.gravity * std::cos(state(0,0)) / params_.length, -params_.friction;
    return res;
}

Pendulum::BMatrix Pendulum::B(XMatrix state, UMatrix controls) {
    (void) state;
    (void) controls;

    Pendulum::BMatrix res;
    res << 0, 1. / (params_.mass * params_.length * params_.length);
    return res;
}

}; /* namespace jcontrols */
}; /* namespace models */