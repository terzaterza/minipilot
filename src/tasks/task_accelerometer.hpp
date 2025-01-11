#pragma once

#include "mp_config.hpp"
#include "emblib/driver/sensor/accelerometer.hpp"
#include "emblib/math/vector.hpp"
#include "emblib/rtos/task.hpp"
#include "emblib/rtos/mutex.hpp"

namespace mp {

class task_accelerometer : public emblib::task {

public:
    explicit task_accelerometer(emblib::accelerometer& accelerometer) :
        task("Task accelerometer", TASK_ACCEL_PRIORITY, m_task_stack),
        m_accelerometer(accelerometer),
        m_last_raw({0, 0, 0}),
        m_last_filtered({0, 0, 0})
    {}

    /**
     * Get last read raw value
     */
    emblib::vector3f get_raw() noexcept
    {
        emblib::scoped_lock lock{m_read_mutex};
        return m_last_raw;
    }

    /**
     * Get last filtered value
     */
    emblib::vector3f get_filtered() noexcept
    {
        emblib::scoped_lock lock{m_read_mutex};
        return m_last_filtered;
    }

private:
    /**
     * Task thread
     */
    void run() noexcept override;

private:
    emblib::task_stack_t<TASK_ACCEL_STACK_SIZE> m_task_stack;
    emblib::accelerometer& m_accelerometer;
    
    emblib::vector3f m_last_raw;
    emblib::vector3f m_last_filtered;
    emblib::mutex m_read_mutex;

};

}