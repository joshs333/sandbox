#ifndef JCONTROLS_DYNAMICS_H
#define JCONTROLS_DYNAMICS_H

#include <functional>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "jmath/utils.h"
#include "jmath/time_range.h"
#include <utility>

#include "jmath/integration/euler.h"

namespace jcontrols {

// Make it easier to format a function for integration
template<class Dynamics, int x_dim = Dynamics::state_dim, int u_dim = Dynamics::control_dim,
typename scalar_T = typename Dynamics::scalar_type, typename UArray = std::vector<Eigen::Matrix<scalar_T,u_dim,1>>,
typename F = std::function<Eigen::Matrix<scalar_T, x_dim, 1>(Eigen::Matrix<scalar_T, x_dim, 1>, double)>>
F intf(Dynamics* dyn, UArray us, jmath::TimeRange range) {
    return [dyn, us, range](Eigen::Matrix<scalar_T, x_dim, 1> x_i, double t_i) {
        return dyn->f(x_i, us[range.getTimeStep(t_i)]);
    };
}

// Make it easier to format a function for integration
template<class Dynamics, int x_dim = Dynamics::state_dim, int u_dim = Dynamics::control_dim,
typename scalar_T = typename Dynamics::scalar_type, typename U = Eigen::Matrix<scalar_T, u_dim, 1>,
typename F = std::function<Eigen::Matrix<scalar_T, x_dim, 1>(Eigen::Matrix<scalar_T, x_dim, 1>, double)>>
F intf(Dynamics* dyn, U u) {
    return [dyn, u](Eigen::Matrix<scalar_T, x_dim, 1> x_i, double t_i) {
        (void) t_i;
        return dyn->f(x_i, u);
    };
}

typedef int dynamics_t;
const int continuous_dynamics = 0;
const int discrete_dynamics = 1;

/**
 * @brief a base dynamics class 
 **/
template<dynamics_t dynamics_T, int state_dimension, int control_dimension, typename scalar_T = double>
class Dynamics {
public:
    //! Dynamics Type
    static const dynamics_t dynamics_type = dynamics_T;
    //! Dimensionality of the state vector (state_dim x 1)
    static const int state_dim = state_dimension;
    //! Dimensionality of the state vector (control_dim x 1)
    static const int control_dim = control_dimension;
    //! Scalar type
    typedef scalar_T scalar_type;
    //! State vector (state_dim x 1)
    typedef Eigen::Matrix<scalar_T, state_dimension, 1> XVector;
    //! Control vector (control_dim x 1)
    typedef Eigen::Matrix<scalar_T, control_dimension, 1> UVector;
    //! A Matrix (state_dim x state_dim)
    typedef Eigen::Matrix<scalar_T, state_dimension, state_dimension> AMatrix;
    //! B vector (state_dim x control_dim)
    typedef Eigen::Matrix<scalar_T, state_dimension, control_dimension> BMatrix;
    //! Linearized Step
    typedef std::pair<AMatrix,BMatrix> StepLin;

    //! Dynamics Function!
    virtual XVector f(XVector state, UVector control) = 0;

    //! Jacobians!
    virtual StepLin taylor(XVector state, UVector control) = 0;
}; /* class Dynamics */

/**
 * @brief a dynamics class based on a function that uses finite differences to get the jacobians
 **/
template<dynamics_t dynamics_T, int state_dimension, int control_dimension, typename scalar_T = double>
class FiniteDifferenceDynamics : public Dynamics<dynamics_T, state_dimension, control_dimension, scalar_T> {
private:
    typedef Dynamics<dynamics_T, state_dimension, control_dimension, scalar_T> BD;
public:
    //! Dynamics function that can be used for this instantiation of dynamics
    typedef std::function<typename BD::XVector(typename BD::XVector, typename BD::UVector)> F;

    //! Create dynamics from a function
    FiniteDifferenceDynamics(F f):
        f_(f)
    {};

    //! Dynamics Function!
    typename BD::XVector f(typename BD::XVector state, typename BD::UVector control) {
        return f_(state, control);
    };

    typename BD::StepLin taylor(typename BD::XVector state, typename BD::UVector control) {
        typename BD::StepLin res;
        res.first = jmath::finite_difference<state_dimension, state_dimension, scalar_T>(
                    std::bind(f_, std::placeholders::_1, control),
                    state
                );

        res.first = jmath::finite_difference<state_dimension, control_dimension, scalar_T>(
            std::bind(f_, state, std::placeholders::_1),
            control
        );
    };

private:
    F f_;
}; /* class FiniteDifferenceDynamics */

/**
 * @brief discretize continuous dynamics
 **/
template<typename IntM, class CD, int state_dimension = CD::state_dim, int control_dimension = CD::control_dim, typename scalar_T = typename CD::scalar_type>
class DiscretizedDynamics : public Dynamics<discrete_dynamics, state_dimension, control_dimension, scalar_T> {
private:
    typedef Dynamics<discrete_dynamics, state_dimension, control_dimension, scalar_T> BD;
public:
    DiscretizedDynamics(CD& cd, double dt = 0.05):
        cd_(cd), dt_(dt)
    {};
    
    //! Dynamics Function!
    typename BD::XVector f(typename BD::XVector state, typename BD::UVector control) {
        return IntM::integrate(std::bind(&CD::f, cd_, std::placeholders::_1, control), state, dt_);
    };

    typename BD::StepLin taylor(typename BD::XVector state, typename BD::UVector control
    ) {
        auto jacs = IntM::template jacobian2<state_dimension,control_dimension>(
            std::bind(&CD::f, cd_, std::placeholders::_1, control),
            std::bind(&CD::taylor, cd_, std::placeholders::_1, control),
            state,
            dt_);
        return jacs;
    };

    double dt() {
        return dt_;
    }

private:
    CD cd_;
    double dt_;
}; /* class DiscretizedDynamics */

}; /* namespace jcontrols */


#endif /* JCONTROLS_DYNAMICS_H */