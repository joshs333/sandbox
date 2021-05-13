#ifndef JCONTROLS_ILQR_H
#define JCONTROLS_ILQR_H

#include "jmath/time_range.h"
#include "jmath/integration/integrate.h"
#include "jmath/integration/identity.h"

namespace jcontrols {
namespace ilqr {

class Solver {
public:
    //! Solver parameters
    typedef struct Params{
        double epsilon_ = 1e-3;
        int max_iters_ = 100;
        Params& epsilon(double e) { epsilon_ = e; return *this; };
        Params& max_iters(int i) { max_iters_ = i; return *this; };
        Params() {};
    } Params;

    Solver(Params params = Params()):
        params_(params)
    {
    };

    Params& params() {
        return params_;
    };

    template<class Problem>
    bool solve(Problem& problem, typename Problem::UTraj& utraj) {
        //! A ton of convenient typedefs (That I might eventually remove idk...)
        const int state_dim = Problem::Dynamics::state_dim;
        const int control_dim = Problem::Dynamics::control_dim;
        typedef typename Problem::Dynamics::scalar_type scalar_type;
        typedef Eigen::Matrix<scalar_type, state_dim, 1> XVector;
        typedef Eigen::Matrix<scalar_type, control_dim, 1> UVector;
        typedef Eigen::Matrix<scalar_type, control_dim, control_dim> UUMatrix;
        typedef Eigen::Matrix<scalar_type, control_dim, state_dim> UXMatrix;
        typedef Eigen::Matrix<scalar_type, state_dim, control_dim> XUMatrix;
        typedef Eigen::Matrix<scalar_type, state_dim, state_dim> XXMatrix;
        typedef typename Problem::XTraj XTraj;
        //! Begine readable code! :)
        auto range = problem.range();
        auto dyn = problem.dynamics();
        auto cf = problem.costfunction();

        // initial xtraj propogation from the seed
        XTraj xtraj(range.size() + 1);
        xtraj[0] = problem.start();
        for(auto it = range.begin(); it != range.end(); ++it) {
            xtraj[it->step + 1] = dyn->f(xtraj[it->step], utraj[it->step]);
        }

        double last_cost = eval_cost(problem, xtraj, utraj);
        std::vector<UXMatrix> K(range.size());
        std::vector<UVector> d(range.size());
        std::vector<XXMatrix> S(range.size() + 1);
        std::vector<XVector> s(range.size() + 1);
        XVector lx, qx, x_k;
        UVector lu, qu, u_k;
        XXMatrix lxx, qxx, qm, A;
        UUMatrix luu, quu, rm;
        UXMatrix qux;
        XUMatrix B;
        // Backward pass
        for(int cycle_idx = 0; cycle_idx < params_.max_iters_; ++cycle_idx) {
            s[range.size()] = cf->Qf() * problem.state(xtraj[range.size()], range.size());
            S[range.size()] = cf->Qf();
            for(auto it = range.rbegin(); it != range.rend(); ++it) {
                // What the hell is going on here you ask>???
                // this: http://roboticexplorationlab.org/papers/iLQR_Tutorial.pdf
                int k = it->step;
                x_k = xtraj[k];
                u_k = utraj[k];
                qm = cf->Q();
                rm = cf->R();
                A = dyn->A(x_k, u_k);
                B = dyn->B(x_k, u_k);
                
                lx = qm * problem.state(x_k, k);
                lu = rm * u_k;
                lxx = qm;
                luu = rm;
                qx = lx + A.transpose() * s[k+1];
                qu = lu + B.transpose() * s[k+1];
                qxx = lxx + A.transpose() * S[k+1] * A;
                quu = luu + B.transpose() * S[k+1] * B;
                qux = B.transpose() * S[k+1] * A;
                d[k] = -quu.inverse() * qu;
                K[k] = -quu.inverse() * qux;
                s[k] = qx + K[k].transpose()*quu*d[k] + K[k].transpose()*qu + qux.transpose()*d[k];
                S[k] = qxx + K[k].transpose()*quu*K[k] + K[k].transpose()*qux + qux.transpose()*K[k];
            }

            // Forward pass (apply update! :) )
            for(auto it = range.begin(); it != range.end(); ++it) {
                utraj[it->step] += K[it->step] * problem.state(xtraj[it->step], it->step) + d[it->step];
                xtraj[it->step + 1] = dyn->f(xtraj[it->step], utraj[it->step]);
            }

            // Check if we have converged
            double new_cost = eval_cost(problem, xtraj, utraj);
            // if (std::fabs((new_cost - last_cost)/last_cost) < params_.epsilon_) {
                // std::cout << "broke after " << cycle_idx << " iterations" << std::endl;
                // break;
                // return true;
            // }
            last_cost = new_cost;
        }

        for(int i = 0; i < xtraj.size(); ++i) {
            std::cout << "step " << i << std::endl << xtraj[i] << std::endl;
        }
        return false;
    };


private:
    template<class Problem>
    double eval_cost(
        Problem& problem,
        typename Problem::XTraj xtraj,
        typename Problem::UTraj utraj
    ) {
        auto range = problem.range();
        auto cf = problem.costfunction();

        double cost = 0;
        for(auto it = range.begin(); it != range.end(); ++it) {
            cost += (problem.state(xtraj[it->step], it->step).transpose() * cf->Q() * problem.state(xtraj[it->step], it->step)
                 + utraj[it->step].transpose() * cf->R() * utraj[it->step])(0,0);
        }
        cost += (problem.state(xtraj[range.size()], range.size()).transpose() * cf->Qf() * problem.state(xtraj[range.size()], range.size()))(0,0);
        return cost;
    };
    Params params_;
}; /* class Solver */


}; /* namespace ilqr */
}; /* namespace jcontrols */

#endif /* JCONTROLS_ILQR_H */