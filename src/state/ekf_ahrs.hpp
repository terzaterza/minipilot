#pragma once

#include "state_estimator.hpp"
#include "emblib/dsp/kalman.hpp"

namespace mp {

/**
 * Extended kalman filter based AHRS estimation
 * @note Does not assume any vehicle physics
 */
class ekf_ahrs : public state_estimator {

    /**
     * Dimension of the state vector used by the kalman filter
     * 3 - acceleration
     * 4 - rotation quaternion
     * 3 - angular velocity
     * 3 - gyro drift
     */
    static constexpr size_t KALMAN_DIM = 13;

    /**
     * Dimension of the measurement vector
     * 3 - accelerometer
     * 3 - gyroscope
     * @todo 3 - magnetometer
     */
    static constexpr size_t OBS_DIM = 6;


    // Convenience typedef
    using state_vec_t = vectorf<KALMAN_DIM>;

public:
    explicit ekf_ahrs() noexcept;

    /**
     * Algorithm iteration
     */
    void update(const sensor_data_s& input, float dt) noexcept override;

    /**
     * Get the current state
     */
    state_s get_state() const noexcept override
    {
        return {
            .position = 0,
            .velocity = 0,
            .acceleration = get_linear_acceleration(m_kalman.get_state()),
            .angular_velocity = get_angular_velocity(m_kalman.get_state()),
            .rotationq = get_rotation_q(m_kalman.get_state())
        };
    }

private:
    /**
     * Kalman filter state transition - `f`
     * @note View docs for this task for reasoning
     */
    state_vec_t state_transition(const state_vec_t& state, float dt) const noexcept;

    /**
     * Kalman filter state transition jacobian - `F`
     * 
     * Represents the derivative of `state_transition` function with respect to the state vector
     */
    matrixf<KALMAN_DIM> state_transition_jacob(const state_vec_t& state, float dt) const noexcept;

    /**
     * Kalman filter state to observation mapping - `h`
     */
    vectorf<OBS_DIM> state_to_obs(const state_vec_t& state, float dt) const noexcept;

    /**
     * Kalman filter state to observation mapping jacobian - `H`
     */
    matrixf<OBS_DIM, KALMAN_DIM> state_to_obs_jacob(const state_vec_t& state, float dt) const noexcept;
    
    // Extract the acceleration vector from the kalman state vector
    static vector3f get_linear_acceleration(const state_vec_t& state) noexcept
    {
        return {state(0), state(1), state(2)};
    }

    // Extract the rotation quaternion from the kalman state vector
    static quaternionf get_rotation_q(const state_vec_t& state) noexcept
    {
        return {state(3), state(4), state(5), state(6)};
    }

    // Extract the angular velocity vector from the kalman state vector
    static vector3f get_angular_velocity(const state_vec_t& state) noexcept
    {
        return {state(7), state(8), state(9)};
    }

    // Extract the gyro drift vector from the kalman state vector
    static vector3f get_gyro_drift(const state_vec_t& state) noexcept
    {
        return {state(10), state(11), state(12)};
    }

private:
    emblib::kalman<KALMAN_DIM> m_kalman;
};

}