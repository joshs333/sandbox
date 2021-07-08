
#ifndef JTRAJ_VELOCITY_PROFILE_H
#define JTRAJ_VELOCITY_PROFILE_H

#include <Eigen/Core>
#include <Eigen/Dense>

namespace jtraj {
namespace velocity_profile {

typedef Eigen::Matrix<double, 2, 1> XYPoint;


template<typename EigenM = XYPoint, int x_idx=0, int y_idx=1>
double segDist(EigenM start, EigenM end) {
    return std::sqrt(
        std::pow(start(x_idx,0) - end(x_idx,0), 2)
        + std::pow(start(y_idx,0) - end(y_idx,0), 2)
    );
}

template<typename EigenM = XYPoint, int x_idx=0, int y_idx=1>
double calcSegmentTime(
    EigenM start, EigenM end,
    double vel, double acc, double target_vel
) {
    XYPoint dist = end - start;
    double distn = std::sqrt(std::pow(dist(x_idx,0),2) + std::pow(dist(y_idx,0),2));

    double det = std::pow(vel,2) + 4*acc*distn;
    double time_to_vel = std::fabs(target_vel - vel) / acc;
    double time_to_end = 0.;
    if (det >= 0) {
        double t1 = (-vel + std::sqrt(det)) / (2.*acc);
        double t2 = (-vel - std::sqrt(det)) / (2.*acc);
        if (t1 > t2) {
            time_to_end = t1;
        } else {
            time_to_end = t2;
        }
    }

    if(det < 0 || time_to_vel < time_to_end) {
        double dist_to_vel = acc * time_to_vel * time_to_vel + vel * time_to_vel;
        double dist_remaining = distn - dist_to_vel;
        double time_remaining = dist_remaining / target_vel;
        return time_to_vel + time_remaining;
    }
    return time_to_end;
}



template<typename EigenM = XYPoint, int x_idx=0, int y_idx=1>
bool profile(std::vector<EigenM> path,
    double start_vel, double target_vel,
    double accel_rate, double decel_rate, double dt,
    std::vector<BM::XMatrix>& t_path, int max_steps
) {
    t_path.clear();

    if(path.size() <= 2) {
        return false;
    }

    double accel = target_vel > start_vel ? accel_rate : decel_rate;
    std::printf("%3.2f\n", accel);

    double cur_vel = start_vel;
    if(std::fabs(target_vel - cur_vel) < 0.001) {
        cur_vel = target_vel;
        accel = 0.0;
    }
    double cur_time = 0.;
    int cur_idx = 0;
    double segment_start_time = 0;
    double segment_end_time = calcSegmentTime(
            path[0], path[1], start_vel,
            accel ,target_vel);


    XYPoint cur_pt = path[0];
    XYPoint end_pt = path[1];
    double seg_dist = segDist(cur_pt, end_pt);
    double seg_interp_dist = 0.0;
    XYPoint last_pt = cur_pt;
    while (true) {
        if(t_path.size() >= max_steps) {
            return true;
        }
        double interp_dist = 0.0;
        // Based on the current velocity and the profile - determine distance to interpolate along the path...
        if(std::fabs(cur_vel - target_vel) > 0.001) {
            // We are still accelerating or decelerating... how long until we reach the target velocity?
            double time_to_velocity = (target_vel - cur_vel)/accel;
            double dist_to_velocity = time_to_velocity * time_to_velocity * accel + time_to_velocity * cur_vel;

            // If we are reaching velocity within this step in time, account for that
            if(time_to_velocity < dt) {
                double time_left = dt - time_to_velocity;
                double dist_after_vel = time_left * target_vel;
                interp_dist = dist_to_velocity + dist_after_vel;
                cur_vel = target_vel;
                accel = 0.;
            // Otherwise we can just have a constance accel / vel in this time step.
            } else {
                double step_dist = dt*dt*accel + dt*cur_vel;
                interp_dist = step_dist;
                cur_vel = cur_vel + dt*accel;
            }
        } else {
            cur_vel = target_vel;
            accel = 0.;
            interp_dist = cur_vel * dt;
        }
        cur_time += dt;

        while(seg_interp_dist + interp_dist > seg_dist) {
            interp_dist -= seg_dist - seg_interp_dist;
            seg_interp_dist = 0;
            if(cur_idx + 2 < path.size()) {
                ++cur_idx;
                cur_pt = path[cur_idx];
                end_pt = path[cur_idx + 1];
                segment_start_time = segment_end_time;
                seg_dist = segDist(cur_pt, end_pt);
            } else {
                std::cout << "Stuff... " << std::endl;
                cur_pt = end_pt;
                XYPoint vec = path[path.size() - 1] - path[path.size() - 2];
                double s_extend;
                if (cur_vel > target_vel){
                    s_extend = cur_vel * dt * 2.;
                } else {
                    s_extend = target_vel * dt * 2.;
                }
                end_pt = cur_pt + vec * (s_extend + seg_dist) / (seg_dist);
                seg_dist = segDist(cur_pt, end_pt);

                std::cout << cur_pt << std::endl;
                std::cout << end_pt << std::endl;
                std::cout << seg_dist << std::endl << std::endl;
            }
        }
        XYPoint interp_pt = cur_pt + (end_pt - cur_pt) * (interp_dist + seg_interp_dist) / seg_dist;
        seg_interp_dist += interp_dist;
        BM::XMatrix new_pt;
        new_pt << interp_pt(0,0), interp_pt(1,0), 0, cur_vel;
        t_path.push_back(new_pt);
        std::cout << segDist(interp_pt, last_pt) << std::endl;
        last_pt = interp_pt;
    }
}


}; /* namespace velocity_profile */
}; /* namespace jtraj */
#endif
