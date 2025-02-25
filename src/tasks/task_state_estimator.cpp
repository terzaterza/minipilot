#include "mp/util/constants.hpp"
#include "task_state_estimator.hpp"
#include "util/logger.hpp"
#include <cmath>

namespace mp {

// Conversion of the task period to floating point delta time
inline constexpr float DT = std::chrono::duration<float>(TASK_STATE_PERIOD).count();

void task_state_estimator::run() noexcept
{
    // Assuming that sensor covariances won't change during runtime
    const matrix3f accel_cov = m_task_accel.get_noise_variance();
    const matrix3f gyro_cov = m_task_gyro.get_noise_variance();
    
    while (true) {
        // Get latest sensor measurements
        vector3f a_read = m_task_accel.get_filtered();
        vector3f w_read = m_task_gyro.get_filtered();
        // TODO: Get rest of the sensors here
        
        sensor_data_s sensor_data {
            .accelerometer = &a_read,
            .accelerometer_cov = &accel_cov,
            .gyroscope = &w_read,
            .gyroscope_cov = &gyro_cov
        };
        m_state_estimator.update(sensor_data, DT);

        // Assign the estimator state to the readable state struct
        m_state_mutex.lock();
        m_state = m_state_estimator.get_state();
        m_state_mutex.unlock();

        sleep_periodic(TASK_STATE_PERIOD);
    }
}

}