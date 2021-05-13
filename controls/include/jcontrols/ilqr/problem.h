#ifndef JCONTROLS_ILQR_ILQR_PROBLEMS_H
#define JCONTROLS_ILQR_ILQR_PROBLEMS_H

#include "jmath/time_range.h"

namespace jcontrols {
namespace ilqr {

template<class DiscretizedDynamics, class CostFunction,
int state_dimension = DiscretizedDynamics::state_dim,
int control_dimension = DiscretizedDynamics::control_dim,
typename scalar_T = typename DiscretizedDynamics::scalar_type>
class Problem {
public:
    typedef DiscretizedDynamics Dynamics;
    typedef CostFunction CFunction;

    virtual typename Dynamics::XMatrix state(typename Dynamics::XMatrix x_k, int k) = 0;
    virtual Dynamics* dynamics() = 0;
    virtual CFunction* costfunction() = 0;
    virtual jmath::TimeRange range() = 0;
}; /* class Problem */

/**
 * iLQR Problem definition to track a single goal
 **/
template<class DiscretizedDynamics, class CostFunction>
class TrackGoalProblem {
public:
    typedef DiscretizedDynamics Dynamics;
    typedef CostFunction CFunction;

    typedef std::vector<typename Dynamics::XMatrix> XTraj;
    typedef std::vector<typename Dynamics::UMatrix> UTraj;

    TrackGoalProblem(
        DiscretizedDynamics* dyn,
        CostFunction* cf,
        int horizon,
        typename Dynamics::XMatrix start,
        typename Dynamics::XMatrix goal    
    ):
        dyn_(dyn),
        cf_(cf),
        start_(start),
        goal_(goal),
        range_(0., horizon, dyn->dt())
    {};

    TrackGoalProblem& updateStart(typename Dynamics::XMatrix start) {
        start_ = start;
        return *this;
    };

    TrackGoalProblem& updateGoal(typename Dynamics::XMatrix goal) {
        goal_ = goal;
        return *this;
    };

    typename Dynamics::XMatrix
    start() {
        return start_;
    }

    typename Dynamics::XMatrix
    state(typename Dynamics::XMatrix x_k, int k) {
        (void) k;
        return x_k - goal_;
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
    typename Dynamics::XMatrix start_;
    typename Dynamics::XMatrix goal_;
    jmath::TimeRange range_;

}; /* class TrackGoalProblem */


/**
 * iLQR Problem definition to track a single goal
 **/
template<class DiscretizedDynamics, class CostFunction>
class TrackTrajProblem {
public:
    typedef DiscretizedDynamics Dynamics;
    typedef CostFunction CFunction;

    typedef std::vector<typename Dynamics::XMatrix> XTraj;
    typedef std::vector<typename Dynamics::UMatrix> UTraj;

    TrackTrajProblem(
        DiscretizedDynamics* dyn,
        CostFunction* cf,
        int horizon,
        typename Dynamics::XMatrix start,
        XTraj traj    
    ):
        dyn_(dyn),
        cf_(cf),
        start_(start),
        traj_(traj),
        range_(0., (int)traj.size() - 1, dyn->dt())
    {};

    TrackTrajProblem& updateStart(typename Dynamics::XMatrix start) {
        start_ = start;
        return *this;
    };

    TrackTrajProblem& updateGoal(XTraj traj) {
        traj_ = traj;
        return *this;
    };

    typename Dynamics::XMatrix
    start() {
        return start_;
    }

    typename Dynamics::XMatrix
    state(typename Dynamics::XMatrix x_k, int k) {
        return x_k - traj_[k];
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
    typename Dynamics::XMatrix start_;
    XTraj traj_;
    jmath::TimeRange range_;

}; /* class TrackGoalProblem */

}; /* namespace ilqr */
}; /* namespace jcontrols */

#endif