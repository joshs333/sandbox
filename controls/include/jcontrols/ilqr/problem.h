#ifndef JCONTROLS_ILQR_ILQR_PROBLEMS_H
#define JCONTROLS_ILQR_ILQR_PROBLEMS_H

#include "jmath/time_range.h"
#include "jmath/utils.h"

namespace jcontrols {
namespace ilqr {

template<class DiscretizedDynamics, class CostFunction>
class Problem {
public:
    typedef DiscretizedDynamics Dynamics;
    typedef CostFunction CFunction;

    virtual typename Dynamics::XVector state(typename Dynamics::XVector x_k, int k) = 0;
    virtual Dynamics* dynamics() = 0;
    virtual CFunction* costfunction() = 0;
    virtual jmath::TimeRange range() = 0;
}; /* class Problem */

/**
 * iLQR Problem definition to track a single goal
 **/
template<class DiscretizedDynamics, class CostFunction>
class TrackTrajProblem {
public:
    typedef DiscretizedDynamics Dynamics;
    typedef CostFunction CFunction;

    typedef std::vector<typename Dynamics::XVector> XTraj;
    typedef std::vector<typename Dynamics::UVector> UTraj;

    TrackTrajProblem(
        DiscretizedDynamics* dyn,
        CostFunction* cf,
        int horizon,
        typename Dynamics::XVector start,
        XTraj traj    
    ):
        dyn_(dyn),
        cf_(cf),
        start_(start),
        traj_(traj),
        range_(0., (unsigned int)traj.size() - 1, dyn->dt())
    {
        ulimit_ = Dynamics::UVector::Ones() * jmath::infinity;
    };

    void setULimits(typename Dynamics::UVector ulimit) {
        ulimit_  = ulimit;
    }

    typename Dynamics::UVector getULimits() {
        return ulimit_;
    }

    TrackTrajProblem& updateStart(typename Dynamics::XVector start) {
        start_ = start;
        return *this;
    };

    TrackTrajProblem& updateGoal(XTraj traj) {
        traj_ = traj;
        return *this;
    };

    typename Dynamics::XVector
    start() {
        return start_;
    }

    typename Dynamics::XVector
    state(typename Dynamics::XVector x_k, int k) {
        typename Dynamics::XVector result;
        result = x_k - traj_[k];
        dyn_->wrap(result);
        return result;
    };

    DiscretizedDynamics* dynamics() {
        return dyn_;
    };

    CostFunction* costfunction() {
        return cf_;
    };
    
    jmath::TimeRange range() {
        return range_;
    };

private:
    DiscretizedDynamics* dyn_;
    CostFunction* cf_;
    typename Dynamics::XVector start_;
    typename Dynamics::UVector ulimit_;
    XTraj traj_;
    jmath::TimeRange range_;

}; /* class TrackGoalProblem */

}; /* namespace ilqr */
}; /* namespace jcontrols */

#endif