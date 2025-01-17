#pragma once

#include "mp_config.hpp"
#include "dynamics/model.hpp"
#include "dynamics/state.hpp"
#include "tasks/task_accelerometer.hpp"
#include "tasks/task_gyroscope.hpp"
#include "emblib/dsp/kalman.hpp"
#include "emblib/rtos/mutex.hpp"
#include "emblib/rtos/task.hpp"

namespace mp {

/**
 * Task responsible for getting the sensor data and estimating the model state
 */
class task_state_estimator : public emblib::task {

public:
    explicit task_state_estimator(
        model& model,
        task_accelerometer& task_accel,
        task_gyroscope& task_gyro
    ) noexcept :
        task("Task state estimator", TASK_STATE_PRIORITY, m_task_stack),
        m_model(&model),
        m_task_accel(task_accel),
        m_task_gyro(task_gyro)
    {}

    // TODO: Add a get_state method which reads from m_state_readable
private:
    /**
     * Dimension of the state vector used by the kalman filter
     * 3 - velocity
     * 3 - acceleration
     * 4 - rotation quaternion
     * 3 - angular velocity
     */
    static constexpr size_t KALMAN_DIM = 13;

    /**
     * Dimension of the measurement vector
     * 3 - accelerometer
     * 3 - gyroscope
     * 
     * @note This dimension does not need to be fixed since we
     * can have a GPS (or some other sensor) measurement every n-th iteration
     */
    static constexpr size_t OBS_DIM = 6;


    // Convenience typedef
    using state_vec_t = emblib::vectorf<KALMAN_DIM>;

private:
    /**
     * Task thread
     */
    void run() noexcept override;

    /**
     * Kalman filter state transition - `f`
     * @note View docs for this task for reasoning
     */
    state_vec_t state_transition(const state_vec_t& state) const noexcept;

    /**
     * Kalman filter state transition jacobian - `F`
     * 
     * Represents the derivative of `state_transition` function with respect to the state vector
     */
    emblib::matrixf<KALMAN_DIM> state_transition_jacob(const state_vec_t& state) const noexcept;

    /**
     * Kalman filter state to observation mapping - `h`
     */
    emblib::vectorf<OBS_DIM> state_to_obs(const state_vec_t& state) const noexcept;

    /**
     * Kalman filter state to observation mapping jacobian - `H`
     */
    emblib::matrixf<OBS_DIM, KALMAN_DIM> state_to_obs_jacob(const state_vec_t& state) const noexcept;

    
    // Extract the velocity vector from the kalman state vector
    static emblib::vector3f get_vel(const state_vec_t& state) noexcept
    {
        return {state(0), state(1), state(2)};
    }
    
    // Extract the acceleration vector from the kalman state vector
    static emblib::vector3f get_acc(const state_vec_t& state) noexcept
    {
        return {state(3), state(4), state(5)};
    }

    // Extract the rotation quaternion from the kalman state vector
    static emblib::quaternionf get_rotq(const state_vec_t& state) noexcept
    {
        return {state(6), state(7), state(8), state(9)};
    }

    // Extract the angular velocity vector from the kalman state vector
    static emblib::vector3f get_ang_vel(const state_vec_t& state) noexcept
    {
        return {state(10), state(11), state(12)};
    }

private:
    emblib::task_stack_t<TASK_STATE_STACK_SIZE> m_task_stack;

    emblib::vector3f m_position = {0};
    emblib::kalman<KALMAN_DIM> m_kalman;
    emblib::mutex m_state_mutex;
    
    model* m_model;
    task_accelerometer& m_task_accel;
    task_gyroscope& m_task_gyro;
};

}