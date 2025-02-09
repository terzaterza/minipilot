#pragma once

#include "mp/util/math.hpp"

namespace mp {

/**
 * State structure used outside of the kalman filter
 * @note The `state` of the kalman filter will be a
 * different type (an N-dimensional vector)
 * @todo Rename velocity to linear_velocity, ...
 */
struct state_s {
    vector3f position {0};
    vector3f velocity {0};
    vector3f acceleration {0};
    vector3f ang_velocity {0};
    quaternionf rotationq {1, 0, 0, 0};
    bool grounded;
};

}