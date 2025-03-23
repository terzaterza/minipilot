#pragma once

#include "task_config.hpp"
#include "mp/util/math.hpp"
#include "util/logger.hpp"
#include "emblib/driver/sensor/three_axis_sensor.hpp"
#include "emblib/rtos/task.hpp"
#include "emblib/rtos/mutex.hpp"

namespace mp {

/**
 * Template task for reading three axis sensors
 * Allows for raw data correction through the `process` method
 */
template <typename data_type>
class task_three_axis_sensor : public emblib::task {

public:
    using vector_t = vector<data_type, 3>;
    using matrix_t = matrix<data_type, 3>;

    explicit task_three_axis_sensor(
        emblib::three_axis_sensor<data_type>& sensor,
        const char* task_name,
        task_priority_e task_priority,
        emblib::ticks_t task_period
    ) :
        task(task_name, task_priority, m_task_stack),
        m_sensor(sensor),
        m_task_period(task_period)
    {}

    /**
     * Get last read raw value
     */
    vector_t get_raw() noexcept
    {
        emblib::scoped_lock lock{m_read_mutex};
        return m_last_raw;
    }

    /**
     * Get last corrected value
     */
    vector_t get_corrected() noexcept
    {
        emblib::scoped_lock lock{m_read_mutex};
        return m_last_corrected;
    }

    /**
     * Get the noise variance matrix based on the sensor noise
     * density and the sampling frequency
     * TODO: This can be moved to the sensor itself, based on
     * the output data rate which it can track
     * TODO: Change the matrix scalar type to data_type/sqrt(Hz)
     */
    matrix3f get_noise_variance() const noexcept
    {
        // TODO: Change this to sensor output data rate
        float fs = 1.f / std::chrono::duration<float>(m_task_period).count();

        float noise_density = m_sensor.get_noise_density();
        float noise_variance = fs * noise_density * noise_density;
        return matrix3f::diagonal(noise_variance);
    }

    /// @todo Add calculate_bias method

private:
    /**
     * Apply processing to the raw input value
     * This can be a filter, bias subtraction, ...
     */
    virtual vector_t process(const vector_t& raw_data) const noexcept = 0;

    /**
     * Task thread
     */
    void run() noexcept override;

private:
    emblib::task_stack_t<512> m_task_stack;
    emblib::ticks_t m_task_period;
    emblib::three_axis_sensor<data_type>& m_sensor;
    
    emblib::mutex m_read_mutex;
    vector_t m_last_raw;
    vector_t m_last_corrected;
};

/**
 * Task implementation
 */
template <typename data_type>
inline void task_three_axis_sensor<data_type>::run() noexcept
{
    // This task should only be created for valid sensors
    // so assert that the sensor is actually working
    assert(m_sensor.probe());

    data_type read_data[3];
    while (true) {
        if (m_sensor.read_all_axes(read_data)) {
            emblib::scoped_lock lock(m_read_mutex);
            m_last_raw = vector_t {read_data[0], read_data[1], read_data[2]};
            m_last_corrected = process(m_last_raw);
        } else {
            // TODO: Add information about sensor type to the log
            log_warning("Sensor reading failed");
        }

        sleep_periodic(m_task_period);
    }
}

}