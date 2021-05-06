#ifndef JCONTROLS_MODELS_PENDULUM_H
#define JCONTROLS_MODELS_PENDULUM_H

#include "jcontrols/dynamics.h"

namespace jcontrols {
namespace models {

class Pendulum : public Dynamics<2,1> {
public:
    struct Params {
        float gravity = 9.8;
        float length = 1;
        float mass = 1;
        float friction = 0;
    };
    Pendulum(Params params);

    XMatrix f(XMatrix state, UMatrix control);
    AMatrix A(XMatrix state, UMatrix control);
    BMatrix B(XMatrix state, UMatrix control);

private:
    Params params_;

}; /* class Pendulum */

}; /* namespace models */
}; /* namespace jcontrols */

#endif /* JCONTROLS_MODELS_PENDULUM_H */