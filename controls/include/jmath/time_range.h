#ifndef JMATH_INTEGRATION_TIME_RANGE_H
#define JMATH_INTEGRATION_TIME_RANGE_H

#include <iterator> // For std::forward_iterator_tag
#include <cstddef>  // For std::ptrdiff_t

#include <iostream>
namespace jmath {

/**
 * @brief expresses a time range with some discretization and some 
 *  functions useful for iterating over that timerange
 **/
class TimeRange {
public:
    struct time_step {
        int step;
        double time;
        double size;
    };
    struct iterator {
        using iterator_category = std::input_iterator_tag;
        using difference_type   = int;
        using value_type        = time_step;
        using pointer           = time_step*;
        using reference         = time_step&;

        iterator(TimeRange* range, int step, int dir = 1):
            range_(range),
            dir_(dir)
        {
            if (step < 0) {
                step_.step = -1;
            } else if (step >= range->size()) {
                step_.step = range->size();
            } else {
                step_ = range->get(step);
            }
        };

        reference operator*() { return step_; };
        pointer operator->() { return &step_; };
        iterator& operator++() { auto new_it = iterator(range_, step_.step + dir_*1); step_ = new_it.step_; return *this; };
        iterator operator++(int) { return iterator(range_, step_.step + dir_*1); };
        friend bool operator== (const iterator& a, const iterator& b) { return a.step_.step == b.step_.step; };
        friend bool operator!= (const iterator& a, const iterator& b) { return a.step_.step != b.step_.step; };    

        private:
            TimeRange* range_;
            time_step step_;
            int dir_;
    };


    TimeRange(double start, double end, double dt);
    TimeRange(double start, int steps, double dt);

    int size();
    iterator begin();
    iterator end();
    iterator rbegin();
    iterator rend();
    time_step get(int i) const;
    double getStepTime(int i) const;
    double getStepSize(int i) const;
    int getTimeStep(double time) const;

private:
    int validateStepNum(int i) const;
    double start_;
    int steps_;
    double dt_;
    double tail_;

}; /* class TimeRange */

}; /* namespace jmath */

#endif /* JMATH_INTEGRATION_TIME_RANGE_H */