#pragma once

#include "emblib/math/vector.hpp"
#include "emblib/math/quaternion.hpp"

namespace mp {

/**
 * State structure used outside of the kalman filter
 * @note The `state` of the kalman filter will be a
 * different type (an N-dimensional vector)
 * @todo Rename velocity to linear_velocity, ...
 */
struct state_s {
    emblib::vector3f position {0};
    emblib::vector3f velocity {0};
    emblib::vector3f acceleration {0};
    emblib::vector3f ang_velocity {0};
    emblib::quaternionf rotationq {1, 0, 0, 0};
};

}