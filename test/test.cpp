#include "drivers/dummy_accel.hpp"
#include "mp/main.hpp"
#include "emblib/driver/io/stdio_dev.hpp"

/**
 * This source file is used to test things on the fly
 * if this is a top level project
 */
int main()
{
    dummy_accel accel;
    emblib::stdio_dev stdio_dev;

    mp::devices_s devices {
        .accelerometer = accel,
        .log_device = &stdio_dev
    };

    return mp::main(devices);
}