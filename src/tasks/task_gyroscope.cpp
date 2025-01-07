#include "./task_gyroscope.hpp"
#include "util/logger.hpp"

namespace mp {

void task_gyroscope::run() noexcept
{
    assert(m_gyroscope.probe());

    float read_data[3];
    while (true) {
        if (m_gyroscope.read_all_axes(read_data)) {
            // TODO: Add ability to remap axis from the sensor to the expected order here
            emblib::vector3f new_raw {read_data[0], read_data[1], read_data[2]};
            /** @todo Compute filtered value here */

            std::scoped_lock lock(m_read_mutex);
            m_last_raw = new_raw;
            /** @todo Assign new filtered value here from the filter */
        } else {
            log_warning("Gyroscope reading failed");
        }

        sleep_periodic(TASK_GYRO_PERIOD);
    }
}

}