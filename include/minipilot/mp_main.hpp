#pragma once

#include "emblib/driver/io/char_dev.hpp"
#include "emblib/driver/sensor/accelerometer.hpp"

namespace mp {

struct mp_devices_s {
    emblib::char_dev* log_device;
    emblib::accelerometer& accelerometer;
};

int mp_main(const mp_devices_s& devices);

}