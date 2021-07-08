#ifndef JTRAJ_TRAJ_PLOTTING_H
#define JTRAJ_TRAJ_PLOTTING_H

#ifndef BUILD_PYTHON
#define BUILD_PYTHON true
#endif

#if BUILD_PYTHON
#include "matplotlib.h"
namespace plt = matplotlibcpp;
#endif

#include <Eigen/Core>
#include <cstdio>


namespace jtraj {
namespace plotting {

template<int x_idx=0, int y_idx=1, typename EigenM = Eigen::Matrix<double,2,1>>
void getXY(std::vector<EigenM>& traj, std::vector<double>& x_vec, std::vector<double>& y_vec) {
    x_vec.clear();
    y_vec.clear();
    for(int i = 0; i < traj.size(); ++i) {
        x_vec.push_back(traj[i](x_idx,0));
        y_vec.push_back(traj[i](y_idx,0));
    }
}

template<int x_idx=0, int y_idx=1, typename EigenM = Eigen::Matrix<double,2,1>>
void plotXY(std::vector<EigenM>& traj, bool plot = true) {
    std::vector<double> x_vec;
    std::vector<double> y_vec;
    getXY<x_idx, y_idx, EigenM>(traj, x_vec, y_vec);

    #if BUILD_PYTHON
        plt::plot(x_vec, y_vec);
        if(plot)
            plt::show();
    #else
        std::printf("Unable to perform plotting call.\n");
        std::printf("X[0]: %4.2f, Y[0]: %4.2f\n", x_vec[0], y_vec[0]);
        std::printf("X[%lu]: %4.2f, Y[%lu]: %4.2f\n", x_vec.size() - 1, x_vec[x_vec.size() - 1], y_vec.size() - 1, y_vec[y_vec.size() - 1]);
    #endif
}

}; /* namespace plotting */
}; /* namespace jtraj */

#endif