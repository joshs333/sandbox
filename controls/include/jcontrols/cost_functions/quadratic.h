#ifndef JCONTROLS_COST_FUNCTIONS_QUADRATIC_H
#define JCONTROLS_COST_FUNCTIONS_QUADRATIC_H

#include "jcontrols/cost_function.h"
#include <Eigen/Core>
#include "jmath/time_range.h"

namespace jcontrols {
namespace cost_functions {

/**
 * @brief a Quadratic Cost function, super simple
 **/
template<int state_dimension, int control_dimension, typename scalar_T = double>
class QuadraticCostFunction : public CostFunction<state_dimension, control_dimension, scalar_T> {
private:
    typedef CostFunction<state_dimension, control_dimension, scalar_T> BCF;
public:
    QuadraticCostFunction(
        std::array<scalar_T, state_dimension> qf,
        std::array<scalar_T, state_dimension> q,
        std::array<scalar_T, control_dimension> r
    ) {
        qf_ << BCF::QMatrix::Identity();
        q_ << BCF::QMatrix::Identity();
        r_ << BCF::RMatrix::Identity();
        for(int i = 0; i < state_dimension; ++i) {
            qf_(i,i) = qf[i];
            q_(i,i) = q[i];
        }
        for(int i = 0; i < control_dimension; ++i) {
            r_(i,i) = r[i];
        }
    };

    QuadraticCostFunction(
        std::array<scalar_T, state_dimension> q,
        std::array<scalar_T, control_dimension> r
    ) {
        qf_ << BCF::QMatrix::Identity();
        q_ << BCF::QMatrix::Identity();
        r_ << BCF::RMatrix::Identity();
        for(int i = 0; i < state_dimension; ++i) {
            qf_(i,i) = q[i];
            q_(i,i) = q[i];
        }
        for(int i = 0; i < control_dimension; ++i) {
            r_(i,i) = r[i];
        }
    };

    QuadraticCostFunction(double q_f, double q, double r) {
        qf_ << BCF::QMatrix::Identity() * q_f;
        q_ << BCF::QMatrix::Identity() * q;
        r_ << BCF::RMatrix::Identity() * r;
    };

    QuadraticCostFunction(double q, double r) {
        qf_ << BCF::QMatrix::Identity() * q;
        q_ << BCF::QMatrix::Identity() * q;
        r_ << BCF::RMatrix::Identity() * r;
    };

    typename BCF::QMatrix taylor_f() {
        return qf_;
    };

    typename BCF::TaylorExpansion taylor() {
        typename BCF::TaylorExpansion res;
        res.first = q_;
        res.second = r_;
        return res;
    };

private:
    typename BCF::QMatrix qf_;
    typename BCF::QMatrix q_;
    typename BCF::RMatrix r_;
}; /* class QuadraticCostFunction */

}; /* namespace cost_functions */
}; /* namespace jcontrols */

#endif /* JCONTROLS_COST_FUNCTIONS_QUADRATIC_H */