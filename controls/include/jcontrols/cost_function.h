#ifndef JCONTROLS_COST_FUNCTION_H
#define JCONTROLS_COST_FUNCTION_H

#include <Eigen/Core>
#include "jmath/time_range.h"

namespace jcontrols {

/**
 * @brief a base cost function class
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

    typedef Eigen::Matrix<scalar_T, state_dimension, state_dimension> QMatrix;
    typedef Eigen::Matrix<scalar_T, control_dimension, control_dimension> RMatrix;
    typedef std::pair<QMatrix,RMatrix> TaylorExpansion;

    //! Final state cost
    virtual QMatrix taylor_f() = 0;

    //! Linearized Cost function
    virtual TaylorExpansion taylor() = 0;
}; /* class CostFunction */

}; /* namespace jcontrols */

#endif /* JCONTROLS_COST_FUNCTION_H */