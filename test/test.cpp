#include "minipilot/mp_main.hpp"
#include "emblib/driver/io/stdio_dev.hpp"

/**
 * This source file is used to test things on the fly
 * if this is a top level project
 */
int main()
{
    emblib::stdio_dev std_dev;
    mp::mp_devices_s devices {
        .log_device = &std_dev
    };
    return mp::mp_main(devices);
}