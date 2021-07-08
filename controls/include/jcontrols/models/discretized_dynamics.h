#ifndef JCONTROLS_MODELS_DISCRETIZED_H
#define JCONTROLS_MODELS_DISCRETIZED_H

#include "jcontrols/dynamics.h"

namespace jcontrols {
namespace models {

/**
 * @brief discretize continuous dynamics
 **/
template<class CD>
class DiscretizedDynamics : public DynamicsBase<dynamics_type_t::discrete, CD::state_dim, CD::control_dim, typename CD::scalar_type> {
private:
    typedef DynamicsBase<dynamics_type_t::discrete, CD::state_dim, CD::control_dim, typename CD::scalar_type> BD;
public:
    DiscretizedDynamics(CD& cd, double dt = 0.05):
        cd_(cd), dt_(dt)
    {}
    
    //! Dynamics Function!
    typename BD::XVector f(typename BD::XVector state, typename BD::UVector control) {
        typename BD::XVector result;
        result = state + cd_.f(state, control) * dt_;
        cd_.wrap(result);
        return result;
    }

    typename BD::TaylorExpansion taylor(typename BD::XVector state, typename BD::UVector control) {
        typename BD::TaylorExpansion res = cd_.taylor(state, control);
        res.first = res.first * dt_;
        res.second = res.second * dt_;
        return res;
    }

private:
    CD cd_;
    double dt_;
}; /* class DiscretizedDynamics */

}; /* namespace models */
}; /* namespace jcontrols */

#endif /* JCONTROLS_MODELS_DISCRETIZED_H */