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
            // TODO: Compute the filtered value here

            emblib::scoped_lock lock(m_read_mutex);
            m_last_raw = new_raw;
            m_last_filtered = new_raw;
            // TODO: Assign the filtered value here
        } else {
            log_warning("Gyroscope reading failed\n");
        }

        sleep_periodic(TASK_GYRO_PERIOD);
    }
}

}