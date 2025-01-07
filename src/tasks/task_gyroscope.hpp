#pragma once

#include "mp_config.hpp"
#include "emblib/driver/sensor/gyroscope.hpp"
#include "emblib/math/vector.hpp"
#include "emblib/rtos/task.hpp"
#include "emblib/rtos/mutex.hpp"
#include <mutex>

namespace mp {

class task_gyroscope : public emblib::task {

public:
    explicit task_gyroscope(emblib::gyroscope& gyroscope) :
        task("Task gyroscope", TASK_GYRO_PRIORITY, m_task_stack),
        m_gyroscope(gyroscope),
        m_last_raw({0, 0, 0}),
        m_last_filtered({0, 0, 0})
    {}

    /**
     * Get last read raw value
     */
    emblib::vector3f get_raw() noexcept
    {
        std::scoped_lock lock{m_read_mutex};
        return m_last_raw;
    }

    /**
     * Get last filtered value
     */
    emblib::vector3f get_filtered() noexcept
    {
        std::scoped_lock lock{m_read_mutex};
        return m_last_filtered;
    }

private:
    /**
     * Task thread
     */
    void run() noexcept override;

private:
    emblib::task_stack_t<TASK_GYRO_STACK_SIZE> m_task_stack;
    emblib::gyroscope& m_gyroscope;
    
    emblib::vector3f m_last_raw;
    emblib::vector3f m_last_filtered;
    emblib::mutex m_read_mutex;

};

}