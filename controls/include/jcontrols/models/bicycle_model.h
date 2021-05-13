#ifndef JCONTROLS_MODELS_PENDULUM_H
#define JCONTROLS_MODELS_PENDULUM_H

#include "jcontrols/dynamics.h"

namespace jcontrols {
namespace models {

class BicycleModel : public Dynamics<4,2> {
public:
    struct Params {
        double L = 1.0;
    };
    BicycleModel(Params params);

    XMatrix f(XMatrix state, UMatrix control);
    AMatrix A(XMatrix state, UMatrix control);
    BMatrix B(XMatrix state, UMatrix control);

private:
    Params params_;

}; /* class BicycleModel */

}; /* namespace models */
}; /* namespace jcontrols */

#endif /* JCONTROLS_MODELS_BICYCLE_MODEL_H */