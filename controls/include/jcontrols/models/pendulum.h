#ifndef JCONTROLS_MODELS_PENDULUM_H
#define JCONTROLS_MODELS_PENDULUM_H

#include "jcontrols/dynamics.h"

namespace jcontrols {
namespace models {

class Pendulum : public Dynamics<continuous_dynamics,2,1> {
public:
    struct Params {
        float gravity = 9.8;
        float length = 1;
        float mass = 1;
        float friction = 0;
    };
    Pendulum(Params params);

    XVector f(XVector state, UVector control);
    StepLin taylor(XVector state, UVector control);

private:
    Params params_;

}; /* class Pendulum */

}; /* namespace models */
}; /* namespace jcontrols */

#endif /* JCONTROLS_MODELS_PENDULUM_H */