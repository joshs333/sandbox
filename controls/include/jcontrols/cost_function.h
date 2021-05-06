#ifndef JCONTROLS_COST_FUNCTION_H
#define JCONTROLS_COST_FUNCTION_H

#include <Eigen/Core>

namespace jcontrols {

/**
 * @brief a base dynamics class 
 **/
template<int state_dimension, int control_dimension, typename scalar_T = double>
class CostFunction {
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
    //! Control vector (control_dim x 1)
    typedef Eigen::Matrix<scalar_T, state_dimension, control_dimension> BMatrix;

    //! Dynamics Function!
    virtual XMatrix f(XMatrix state, UMatrix control) = 0;

    //! Jacobians!
    virtual AMatrix A(XMatrix state, UMatrix control) = 0;
    virtual BMatrix B(XMatrix state, UMatrix control) = 0;
}; /* class Dynamics */

}; /* namespace jcontrols */

#endif /* JCONTROLS_COST_FUNCTION_H */