#ifndef JCONTROLS_MODELS_BICYCLE_MODEL_H
#define JCONTROLS_MODELS_BICYCLE_MODEL_H

#include "jcontrols/dynamics.h"

namespace jcontrols {
namespace models {

class KinematicBicycleModel : public DynamicsBase<dynamics_type_t::continuous, 4,2> {
public:
    //! Parmaters of the simple kinematic bicycle model
    struct Params {
        //! Length of the base rear to front axle
        double L = 1.0;
    };
    //! Generate the bicycle model
    KinematicBicycleModel(Params params);
    //! Dynamics Function
    XVector f(XVector state, UVector control);
    //! Tayler expansion of the dynamics function
    TaylorExpansion taylor(XVector state, UVector control);
    //! Wrap the yaw
    void wrap(XVector& state);

private:
    //! This instances parameters
    Params params_;

}; /* class KinematicBicycleModel */

}; /* namespace models */
}; /* namespace jcontrols */

#endif /* JCONTROLS_MODELS_BICYCLE_MODEL_H */