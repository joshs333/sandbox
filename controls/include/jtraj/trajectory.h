#ifndef JTRAJ_TRAJECTORY_H
#define JTRAJ_TRAJECTORY_H

#include <array>
#include <string>

namespace jtraj {

template<int dim, std::array<std::string, 1> fields>
class Description {
public:
    //! Eigen vector type
    typedef Eigen::Matrix<double, dim, 1> XVector;
    //! State Dimension
    static const int state_dim = dim;
    //! Field Descriptions
    static const std::array<std::string, dim> state_fields = fields;

    //! Used to check if a field exists
    bool hasField(std::string field) {
        auto idx = getFieldIdx(field);
        if(idx < 0) {
            return false;
        }
        return true;
    };

    //! Used to get the idx for a given field
    int getFieldIdx(std::string field) {
        for(int i = 0; i < dim; ++i) {
            if(state_fields[i] == field) {
                return i;
            }
        }
        return -1;
    };
}; /* class State */


}; /* namespace jtraj */



#endif