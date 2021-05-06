#include "jmath/time_range.h"

#include <exception>
#include <string>

namespace jmath {

TimeRange::TimeRange(double start, double end, double dt) {
    start_ = start;
    steps_ = (end - start) / dt;
    tail_ = end - (steps_ * dt + start);
    // If there is a tail- add an extra step
    if (tail_ > 1e-8) {
        steps_++;
    } else {
        tail_ = 0.0;
    }
    dt_ = dt;
}

TimeRange::TimeRange(double start, int steps, double dt) {
    start_ = start;
    steps_ = steps;
    tail_ = 0.0;
    dt_ = dt;
}

int TimeRange::size() {
    return steps_;
}

TimeRange::iterator TimeRange::begin() {
    return TimeRange::iterator(this, 0);
}

TimeRange::iterator TimeRange::end() {
    return TimeRange::iterator(this, steps_);
}

TimeRange::iterator TimeRange::rbegin() {
    return TimeRange::iterator(this, steps_ - 1, -1);
}

TimeRange::iterator TimeRange::rend() {
    return TimeRange::iterator(this, -1, -1);
}

TimeRange::time_step TimeRange::get(int i) const {
    time_step ts;
    i = validateStepNum(i);
    ts.step = i;
    ts.time = getStepTime(i);
    ts.size = getStepSize(i);
    return ts;
}

double TimeRange::getStepTime(int i) const {
    i = validateStepNum(i);
    return start_ + i * dt_;
}

double TimeRange::getStepSize(int i) const {
    i = validateStepNum(i);
    if (i == steps_ - 1 && tail_ > 1e-8) {
        return tail_;
    }
    return dt_;
}

int TimeRange::getTimeStep(double time) const {
    double range_time = time - start_;
    double end_time = start_ + steps_ * dt_ + tail_;
    if (tail_ > 1e-8) {
        end_time -= dt_;
    }
    if(range_time > end_time) {
        throw std::out_of_range("TimeRange is " + std::to_string(start_) + " to " + std::to_string(end_time) + ", " + std::to_string(time) + " is out of range.");
    }
    return range_time / dt_;
}

int TimeRange::validateStepNum(int i) const {
    if (i < 0 || i >= steps_) {
        throw std::out_of_range("TimeRange has only " + std::to_string(steps_) + " steps, " + std::to_string(i) + " is out of range.");
    }
    return i;
}

}; /* namespace jmath */
