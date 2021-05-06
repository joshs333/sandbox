#include <cstdio>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "jcontrols/dynamics.h"
#include "jcontrols/models/pendulum.h"
#include "jmath/integration/integrate.h"
#include "jmath/time_range.h"
#include <iostream>
#include <chrono>
#include <vector>
#include <array>

using namespace jcontrols;
using namespace jcontrols::models;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

typedef jcontrols::DiscretizedDynamics<jcontrols::models::Pendulum, jmath::integration::Euler> DPD;
typedef jcontrols::models::Pendulum CPD;
typedef jmath::integration::Euler CIntM;
typedef jmath::integration::Euler DIntM;
// https://bjack205.github.io/papers/AL_iLQR_Tutorial.pdf
// https://arxiv.org/pdf/2007.14492.pdf
// https://homes.cs.washington.edu/~todorov/papers/LiICINCO04.pdf

// void forward

const double dt = 0.1;
const int horizon = 10;

template<class Dynamics>
class iLQR {
public:
    static const int x_dim = Dynamics::state_dim;
    static const int u_dim = Dynamics::control_dim;
    typedef typename Dynamics::scalar_type scalar_type;

    typedef Eigen::Matrix<scalar_type, x_dim, 1> XV;
    typedef Eigen::Matrix<scalar_type, u_dim, 1> UV;
    typedef Eigen::Matrix<scalar_type, x_dim, x_dim> AM;
    typedef Eigen::Matrix<scalar_type, x_dim, u_dim> BM;
    typedef Eigen::Matrix<scalar_type, x_dim, x_dim> QM;
    typedef Eigen::Matrix<scalar_type, u_dim, u_dim> RM;


    typedef std::vector<XV> XT;
    typedef std::vector<UV> CT;

    iLQR(Dynamics* dyn, int horizon, double dt):
        dyn_(dyn),
        horizon_(horizon),
        dt_(dt)
    {
    };

    void solve(
        XV start,
        CT seed,
        QM q,
        RM r
    ) {
        jmath::TimeRange range(0., horizon_, dt_);
        auto traj = jmath::integration::integrate<CIntM>(
            jcontrols::intf(dyn_, seed, range),
            start,
            range
        );
        CT us = seed;

        typedef Eigen::Matrix<scalar_type, u_dim, u_dim> BB;
        typedef Eigen::Matrix<scalar_type, u_dim, x_dim> K;
        typedef Eigen::Matrix<scalar_type, u_dim, u_dim> Ku;
        typedef Eigen::Matrix<scalar_type, u_dim, x_dim> Kv;
        typedef Eigen::Matrix<scalar_type, x_dim, x_dim> S;
        typedef Eigen::Matrix<scalar_type, x_dim, 1> V;

        QM sn = q;
        std::vector<V> vs(horizon_ + 1);
        std::vector<K> ks(horizon_);
        std::vector<Ku> kus(horizon_);
        std::vector<Kv> kvs(horizon_);
        vs[-1] = q * traj[-1];
        for(auto it = range.rbegin(); it != range.rend(); ++it) {
            auto A = dyn_->A(traj[it->step], us[it->step]);
            auto B = dyn_->B(traj[it->step], us[it->step]);
            BB b = (B.transpose() * sn * B + r).inverse();

            ks[it->step] = b * B

        }
    };

private:
    Dynamics* dyn_;
    int horizon_;
    double dt_;


};

int main(int argc, char** argv) {
    (void) argc;
    (void) argv;


    DPD::UMatrix zero_control(0.);
    std::vector<DPD::UMatrix> us(horizon);
    for(int i = 0; i < horizon; ++i) {
        us[i] = zero_control;
    }

    DPD::XMatrix start;
    start << M_PI, 0;
    DPD::XMatrix goal;
    goal << 0, 0;

    auto Q = Eigen::Matrix<double, 2, 2>::Identity();
    auto R = Eigen::Matrix<double, 1, 1>::Identity();

    CPD::Params pendulum_params;
    CPD continuous_pendulum(pendulum_params);
    DPD discretized_pendulum(continuous_pendulum, dt);

    iLQR<DPD> ilqr(&discretized_pendulum, horizon, dt);
    ilqr.solve(start, us, Q, R);

    return 0;
}