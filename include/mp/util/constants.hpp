#pragma once

#include "mp/util/math.hpp"
#include "emblib/driver/sensor/accelerometer.hpp"

namespace mp {

// Model direction definitions
inline const vector3f FORWARD    = {1, 0, 0};
inline const vector3f LEFT       = {0, 1, 0};
inline const vector3f UP         = {0, 0, 1};
inline const vector3f BACKWARD   = -FORWARD;
inline const vector3f RIGHT      = -LEFT;
inline const vector3f DOWN       = -UP;

// Gravity const G = 9.80665
inline constexpr float G = emblib::accelerometer::G_TO_MPS2;
// Gravity vector = G * DOWN
inline const vector3f GV = DOWN * G;

}