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
    struct {
        emblib::accelerometer& sensor;
        // Map the sensor reading to the mp coordinate frame
        const matrix3f& transform;
    } accelerometer;
    struct {
        emblib::gyroscope& sensor;
        // Map the sensor reading to the mp coordinate frame
        const matrix3f& transform;
    } gyroscope;
    emblib::char_dev* log_device;
    emblib::char_dev* telemetry_device;
    emblib::char_dev& receiver_device;
};

/**
 * Minipilot entry point
 */
int main(
    const devices_s& devices,
    state_estimator& state_estimator,
    vehicle& vehicle
);

}