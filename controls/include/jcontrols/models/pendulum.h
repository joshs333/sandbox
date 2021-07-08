#ifndef JCONTROLS_MODELS_PENDULUM_H
#define JCONTROLS_MODELS_PENDULUM_H

#include "jcontrols/dynamics.h"

namespace jcontrols {
namespace models {

class Pendulum : public DynamicsBase<dynamics_type_t::continuous,2,1> {
public:
    //! Parameters For the Pendulum
    struct Params {
        float gravity = 9.8;
        float length = 1;
        float mass = 1;
        float friction = 0;
    };
    //! Create a pendulum with these parameters
    Pendulum(Params params);
    //! Pendulum dynamics function
    XVector f(XVector state, UVector control);
    //! Taylor expansion of the dynamics
    TaylorExpansion taylor(XVector state, UVector control);

private:
    //! This instances parameters
    Params params_;

}; /* class Pendulum */

}; /* namespace models */
}; /* namespace jcontrols */

#endif /* JCONTROLS_MODELS_PENDULUM_H */