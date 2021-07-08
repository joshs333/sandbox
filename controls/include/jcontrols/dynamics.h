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

enum class dynamics_type_t {
    continuous,
    discrete
};

/**
 * @brief a base dynamics class 
 **/
template<dynamics_type_t dynamics_T, int state_dimension, int control_dimension, typename scalar_T = double>
class DynamicsBase {
public:
    //! Dimensionality of the state vector (state_dim x 1)
    static const int state_dim = state_dimension;
    //! Dimensionality of the state vector (control_dim x 1)
    static const int control_dim = control_dimension;
    //! Dynamics Type
    static const dynamics_type_t dynamics_type = dynamics_T;
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
    //! Taylor Expansion
    typedef std::pair<AMatrix, BMatrix> TaylorExpansion;

    //! Dynamics Function!
    virtual XVector f(XVector state, UVector control) = 0;
    //! Jacobians
    virtual TaylorExpansion taylor(XVector state, UVector control) = 0;

    //! Make sure any angles get wrapped / etc..
    void wrap(XVector& state) {};
    //! For discrete dynamics, have a function to get the dt
    double dt() { return dt_; };
protected:
    double dt_;
}; /* class Dynamics */


}; /* namespace jcontrols */


#endif /* JCONTROLS_DYNAMICS_H */