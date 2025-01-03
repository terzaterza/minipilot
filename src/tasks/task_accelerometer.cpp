#include "./task_accelerometer.hpp"
#include "util/logger.hpp"

namespace mp {

void task_accelerometer::run() noexcept
{
    assert(m_accelerometer.probe());

    float read_data[3];
    while (true) {
        if (m_accelerometer.read_all_axes(read_data)) {
            emblib::vector3f new_raw {read_data[0], read_data[1], read_data[2]};
            /** @todo Compute filtered value here */

            std::scoped_lock lock(m_read_mutex);
            m_last_raw = new_raw;
            /** @todo Assign new filtered value here from the filter */
        } else {
            log_warning("Accelerometer reading failed");
        }

        sleep_periodic(TASK_ACCEL_PERIOD);
    }
}

}