#ifndef JCONTROLS_MODELS_FINITE_DIFFERENCE_H
#define JCONTROLS_MODELS_FINITE_DIFFERENCE_H

namespace jcontrols {
namespace models {

/**
 * @brief a dynamics class based on a function that uses finite differences to get the jacobians
 **/
template<int state_dimension, int control_dimension, typename scalar_T = double>
class FiniteDifferenceDynamics : public DynamicsBase<state_dimension, control_dimension, scalar_T> {
private:
    typedef DynamicsBase<state_dimension, control_dimension, scalar_T> BD;
public:
    //! Dynamics function that can be used for this instantiation of dynamics
    typedef std::function<BD::XVector(BD::XVector, BD::UVector)> F;

    //! Create dynamics from a function
    FiniteDifferenceDynamics(F f):
        f_(f)
    {};

    //! Dynamics Function!
    BD::XVector f(BD::XVector state, BD::UVector control) {
        return f_(state, control);
    };

    //! Jacobians!
    BD::TaylorExpansion taylor(BD::XVector state, BD::UVector control) {
        BD::TaylorExpansion result;
        result.first = jmath::finite_difference<state_dimension, state_dimension, scalar_T>(
            std::bind(f_, std::placeholders::_1, control),
            state
        );
        result.second = jmath::finite_difference<state_dimension, control_dimension, scalar_T>(
            std::bind(f_, state, std::placeholders::_1),
            control
        );
    };

private:
    F f_;
}; /* class FiniteDifferenceDynamics */

#endif /* JCONTROLS_MODELS_FINITE_DIFFERENCE_H */