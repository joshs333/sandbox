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
    //! State  (state_dim x 1)
    typedef Eigen::Matrix<scalar_T, state_dimension, 1> XVector;
    typedef Eigen::Matrix<scalar_T, control_dimension, 1> UVector;
    typedef Eigen::Matrix<scalar_T, state_dimension, state_dimension> QMatrix;
    // typedef Eigen::Matrix<scalar_T, control_dimension, 1> QVector;
    //! A Matrix (state_dim x state_dim)
    typedef Eigen::Matrix<scalar_T, control_dimension, control_dimension> RMatrix;
    // typedef Eigen::Matrix<scalar_T, state_dimension, control_dimension> RVector;

    //! Final state cost
    virtual QMatrix Qf() = 0;
    // virtual XMatrix qf() = 0;

    //! Linearized Cost function
    virtual QMatrix Q() = 0;
    // virtual XMatrix q() = 0;

    //! Jacobians!
    virtual RMatrix R() = 0;
    // virtual BMatrix r() = 0;


}; /* class CostFunction */

/**
 * @brief a Quadratic Cost function, super simple
 **/
template<int state_dimension, int control_dimension, typename scalar_T = double>
class QuadraticCostFunction : public CostFunction<state_dimension, control_dimension, scalar_T> {
public:
    // Easist way to succinctly get matrix defs
    typedef CostFunction<state_dimension, control_dimension, scalar_T> CFT;
    QuadraticCostFunction(
        typename CFT::QMatrix q,
        typename CFT::RMatrix r,
        typename CFT::QMatrix qf
    ):
        q_(q),
        r_(r),
        qf_(qf)
    {};

    typename CFT::QMatrix Qf() {
        return qf_;
    }
    typename CFT::QMatrix Q() {
        return q_;
    }
    typename CFT::RMatrix R() {
        return r_;
    }

private:
    typename CFT::QMatrix q_;
    typename CFT::RMatrix r_;
    typename CFT::QMatrix qf_;
}; /* class QuadraticCostFunction */


template<int state_dim, int control_dim, typename scalar_T = double>
QuadraticCostFunction<state_dim, control_dim, scalar_T>
QuickQuadraticCost(
    std::array<scalar_T, state_dim> q,
    std::array<scalar_T, control_dim> r,
    std::array<scalar_T, state_dim> qf
) {
    typedef CostFunction<state_dim, control_dim, scalar_T> CFT;
    typename CFT::QMatrix qm, qfm;
    typename CFT::RMatrix rm;

    qm << CFT::QMatrix::Identity();
    qfm << CFT::QMatrix::Identity();
    rm << CFT::RMatrix::Identity();
    for(int i = 0; i < state_dim; ++i) {
        qm(i,i) = q[i];
        qfm(i,i) = qf[i];
    }
    for(int i = 0; i < control_dim; ++i) {
        rm(i,i) = r[i];
    }
    return QuadraticCostFunction<state_dim, control_dim, scalar_T>(qm, rm, qfm);
}

template<int state_dim, int control_dim, typename scalar_T = double>
QuadraticCostFunction<state_dim, control_dim, scalar_T>
QuickQuadraticCost(double q, double r, double q_f) {
    typedef CostFunction<state_dim, control_dim, scalar_T> CFT;
    typename CFT::QMatrix qm, qfm;
    typename CFT::RMatrix rm;

    qm << CFT::QMatrix::Identity() * q;
    qfm << CFT::QMatrix::Identity() * q_f;
    rm << CFT::RMatrix::Identity() * r;
    return QuadraticCostFunction<state_dim, control_dim, scalar_T>(qm, rm, qfm);
}

}; /* namespace jcontrols */

#endif /* JCONTROLS_COST_FUNCTION_H */