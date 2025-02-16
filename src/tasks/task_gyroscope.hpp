#pragma once

#include "mp_config.hpp"
#include "mp/util/math.hpp"
#include "emblib/driver/sensor/gyroscope.hpp"
#include "emblib/rtos/task.hpp"
#include "emblib/rtos/mutex.hpp"

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

    /**
     * Get the noise variance matrix based on the gyroscope noise
     * density and the sampling frequency
     * TODO: This can be moved to the gyroscope itself, based on
     * the output data rate which it can track
     */
    matrix3f get_noise_variance() const noexcept
    {
        // TODO: Change this to gyroscopes output data rate
        constexpr float FS = 1.f / std::chrono::duration<float>(TASK_GYRO_PERIOD).count();

        float noise_density = m_gyroscope.get_noise_density();
        float noise_variance = FS * noise_density * noise_density;
        return vector3f({noise_variance, noise_variance, noise_variance}).as_diagonal();
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