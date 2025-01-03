#pragma once

#include "emblib/driver/io/char_dev.hpp"
#include "emblib/driver/sensor/accelerometer.hpp"

namespace mp {

/**
 * Device drivers required by minipilot
 * 
 * @note Drivers taken as a pointer are optional,
 * and `nullptr` can be passed.
 */
struct devices_s {
    emblib::accelerometer& accelerometer;
    emblib::char_dev* log_device;
};

/**
 * Minipilot entry point
 */
int main(const devices_s& devices);

}