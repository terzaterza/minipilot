#pragma once

#include "mp/util/math.hpp"
#include "emblib/driver/sensor/accelerometer.hpp"

namespace mp {

// Model direction definitions
static inline const vector3f FORWARD    = {1, 0, 0};
static inline const vector3f LEFT       = {0, 1, 0};
static inline const vector3f UP         = {0, 0, 1};
static inline const vector3f BACKWARD   = -FORWARD;
static inline const vector3f RIGHT      = -LEFT;
static inline const vector3f DOWN       = -UP;

// Gravity const G = 9.80665
static inline constexpr float G = emblib::accelerometer::G_TO_MPS2;
// Gravity vector = G * DOWN
static inline const vector3f GV = DOWN * G;

}