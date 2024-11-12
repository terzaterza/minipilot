#pragma once

#include "emblib/math/vector.hpp"
#include "emblib/math/quaternion.hpp"

namespace mp {

/**
 * State structure used outside of the kalman filter
 * @note The `state` of the kalman filter will be a
 * different type (an N-dimensional vector)
 */
struct state_s {
    emblib::vector3f position;
    emblib::vector3f velocity;
    emblib::vector3f ang_velocity;
    emblib::quaternionf rotationq;
};

}