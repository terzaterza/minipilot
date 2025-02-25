#pragma once

#include "vehicles/vehicle.hpp"
#include "emblib/driver/io/char_dev.hpp"
#include "emblib/driver/sensor/accelerometer.hpp"
#include "emblib/driver/sensor/gyroscope.hpp"

namespace mp {

/**
 * Device drivers required by minipilot
 * 
 * @note Drivers taken as a pointer are optional,
 * and `nullptr` can be passed.
 */
struct devices_s {
    emblib::accelerometer& accelerometer;
    emblib::gyroscope& gyroscope;
    emblib::char_dev* log_device;
    emblib::char_dev* telemetry_device;
    emblib::char_dev& receiver_device;
};

/**
 * Minipilot entry point
 */
int main(const devices_s& devices, vehicle& vehicle);

}