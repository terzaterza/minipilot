#pragma once

#include "task_config.hpp"
#include "state/state_estimator.hpp"
#include "tasks/task_accelerometer.hpp"
#include "tasks/task_gyroscope.hpp"
#include "emblib/rtos/mutex.hpp"
#include "emblib/rtos/task.hpp"

namespace mp {

/**
 * Task responsible for getting the sensor data and estimating the model state
 */
class task_state_estimator : public emblib::task {

public:
    // TODO: Add an initial state parameter
    explicit task_state_estimator(
        state_estimator& state_estimator,
        task_accelerometer& task_accel,
        task_gyroscope& task_gyro
    ) noexcept :
        task("Task state estimator", TASK_STATE_PRIORITY, m_task_stack),
        m_state_estimator(state_estimator),
        m_task_accel(task_accel),
        m_task_gyro(task_gyro)
    {}

    /**
     * Get the current state
     * @todo Maybe return as reference
     */
    state_s get_state() noexcept
    {
        emblib::scoped_lock lock(m_state_mutex);
        return m_state;
    }

private:
    /**
     * Task thread
     */
    void run() noexcept override;

private:
    emblib::task_stack_t<TASK_STATE_STACK_SIZE> m_task_stack;

    state_s m_state;
    state_estimator& m_state_estimator;
    emblib::mutex m_state_mutex;
    
    task_accelerometer& m_task_accel;
    task_gyroscope& m_task_gyro;
};

}