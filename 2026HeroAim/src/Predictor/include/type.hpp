#pragma once

#include "utils/include/EigenCompat.hpp"
#include <Eigen/Core>
#include <array>
#include <vector>
#include "utils/include/Location.hpp"

namespace predictor {

struct Armor {
    enum armor_status {
        NONEXIST,
        UNSEEN,
        AVAILABLE
    };

    XYZ center;
    double yaw = 0.0;
    double theta = 0.0;
    int id = -1;
    armor_status status = NONEXIST;
};

struct Prediction {
    XYZ center;
    int id = -1;
    double vx = 0.0;
    double vy = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    double z1 = 0.0;
    double z2 = 0.0;
    double theta = 0.0;
    double omega = 0.0;
    double r1 = 0.0;
    double r2 = 0.0;
    std::array<Armor, 4> armors;
    bool stable = true;
};

using Predictions = std::vector<Prediction>;

} // namespace predictor
