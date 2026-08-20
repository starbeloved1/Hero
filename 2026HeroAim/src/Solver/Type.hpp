#pragma once
#include "utils/include/EigenCompat.hpp"
#include <Eigen/Core>
#include <vector>
#include <array>
#include <cmath>
#include "utils/include/Location.hpp"

namespace Eigen {
    // define Eigen5d
    typedef Eigen::Matrix<double, 5, 1> Vector5d;
}

namespace solver
{
    struct ImuData
    {
        float pitch;
        float yaw;
        float roll;
        ImuData() {};
        ImuData(float p, float y, float r) : pitch(p), yaw(y), roll(r) {};
        ImuData(const PYD& pyd) : pitch(pyd.pitch * 180 / M_PI), yaw(pyd.yaw * 180 / M_PI), roll(0) {};
        operator PYD() const
        {
            return PYD(pitch * M_PI / 180, yaw * M_PI / 180, 0);
        };
    };

}
