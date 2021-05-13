#ifndef JCONTROLS_DYNAMICS_H
#define JCONTROLS_DYNAMICS_H

#include <functional>
#include <vector>
#include <Eigen/Core>
#include "jmath/utils.h"
#include "jmath/time_range.h"

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



/**
 * @brief a base dynamics class 
 **/
template<int state_dimension, int control_dimension, typename scalar_T = double>
class Dynamics {
public:
    //! Dimensionality of the state vector (state_dim x 1)
    static const int state_dim = state_dimension;
    //! Dimensionality of the state vector (control_dim x 1)
    static const int control_dim = control_dimension;
    //! Scalar type
    typedef scalar_T scalar_type;
    //! State vector (state_dim x 1)
    typedef Eigen::Matrix<scalar_T, state_dimension, 1> XMatrix;
    //! Control vector (control_dim x 1)
    typedef Eigen::Matrix<scalar_T, control_dimension, 1> UMatrix;
    //! A Matrix (state_dim x state_dim)
    typedef Eigen::Matrix<scalar_T, state_dimension, state_dimension> AMatrix;
    //! B vector (state_dim x control_dim)
    typedef Eigen::Matrix<scalar_T, state_dimension, control_dimension> BMatrix;
    //! 

    //! Dynamics Function!
    virtual XMatrix f(XMatrix state, UMatrix control) = 0;

    //! Jacobians!
    virtual AMatrix A(XMatrix state, UMatrix control) = 0;
    virtual BMatrix B(XMatrix state, UMatrix control) = 0;
}; /* class Dynamics */

/**
 * @brief a dynamics class based on a function that uses finite differences to get the jacobians
 **/
template<int state_dimension, int control_dimension, typename scalar_T = double>
class FiniteDifferenceDynamics : public Dynamics<state_dimension, control_dimension, scalar_T> {
private:
    typedef Dynamics<state_dimension, control_dimension, scalar_T> BD;
public:
    //! Dynamics function that can be used for this instantiation of dynamics
    typedef std::function<Eigen::Matrix<scalar_T, state_dimension, 1>(
                Eigen::Matrix<scalar_T, state_dimension, 1>,
                Eigen::Matrix<scalar_T, control_dimension, 1>)> F;
    //! Create dynamics from a function
    FiniteDifferenceDynamics(F f):
        f_(f)
    {};

    //! Dynamics Function!
    Eigen::Matrix<scalar_T, state_dimension, 1> f(
        Eigen::Matrix<scalar_T, state_dimension, 1> state,
        Eigen::Matrix<scalar_T, control_dimension, 1> control
    ) {
        return f_(state, control);
    };

    //! Jacobians!
    Eigen::Matrix<scalar_T, state_dimension, state_dimension> A(
        Eigen::Matrix<scalar_T, state_dimension, 1> state,
        Eigen::Matrix<scalar_T, control_dimension, 1> control
    ) {
        return jmath::finite_difference<state_dimension, state_dimension, scalar_T>(
            std::bind(f_, std::placeholders::_1, control),
            state
        );
    };

    Eigen::Matrix<scalar_T, state_dimension, control_dimension> B(
        Eigen::Matrix<scalar_T, state_dimension, 1> state,
        Eigen::Matrix<scalar_T, control_dimension, 1> control
    ) {
        return jmath::finite_difference<state_dimension, control_dimension, scalar_T>(
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
template<class CD, typename Int, int state_dimension = CD::state_dim, int control_dimension = CD::control_dim, typename scalar_T = double>
class DiscretizedDynamics : public Dynamics<state_dimension, control_dimension, scalar_T> {
public:
    DiscretizedDynamics(CD& cd, double dt = 0.05):
        cd_(cd), dt_(dt)
    {};
    
    //! Dynamics Function!
    Eigen::Matrix<scalar_T, state_dimension, 1> f(
        Eigen::Matrix<scalar_T, state_dimension, 1> state,
        Eigen::Matrix<scalar_T, control_dimension, 1> control
    ) {
        return Int::integrate(std::bind(&CD::f, cd_, std::placeholders::_1, control), state, dt_);
    };

    //! Jacobians!
    Eigen::Matrix<scalar_T, state_dimension, state_dimension> A(
        Eigen::Matrix<scalar_T, state_dimension, 1> state,
        Eigen::Matrix<scalar_T, control_dimension, 1> control
    ) {
        Eigen::Matrix<scalar_T, state_dimension, state_dimension> r;
        r << Int::template jacobian<state_dimension>(
            std::bind(&CD::f, cd_, std::placeholders::_1, control),
            std::bind(&CD::A, cd_, std::placeholders::_1, control),
            state,
            dt_);
        return r;
    };

    Eigen::Matrix<scalar_T, state_dimension, control_dimension> B(
        Eigen::Matrix<scalar_T, state_dimension, 1> state,
        Eigen::Matrix<scalar_T, control_dimension, 1> control
    ) {
        auto jacs = Int::template jacobian<state_dimension,control_dimension>(
            std::bind(&CD::f, cd_, std::placeholders::_1, control),
            std::bind(&CD::A, cd_, std::placeholders::_1, control),
            std::bind(&CD::B, cd_, std::placeholders::_1, control),
            state,
            dt_);
        return jacs.second;
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