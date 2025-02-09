#pragma once

#include "mp/vehicles/state.hpp"
#include "mp/util/math.hpp"
#include "emblib/driver/sensor/accelerometer.hpp"

namespace mp {

/**
 * Model of the vehicle with respect to dynamics
 */
class model {

public:
    // Model direction definitions
    static inline const vector3f FORWARD    = {1, 0, 0};
    static inline const vector3f LEFT       = {0, 1, 0};
    static inline const vector3f UP         = {0, 0, 1};
    static inline const vector3f BACKWARD   = -FORWARD;
    static inline const vector3f RIGHT      = -LEFT;
    static inline const vector3f DOWN       = -UP;

    // Gravity vector definition
    static inline constexpr float GRAVITY_CONST = emblib::accelerometer::G_TO_MPS2;
    static inline const vector3f GRAVITY        = DOWN * GRAVITY_CONST;

    // Strucutre contatining all derivatives of `acc` and `ang_acc` with respect to state
    // TODO: Rename fields
    struct jacobian_s {
        matrix3f acc_vel; // d(acc)/d(vel)
        matrixf<3, 4> acc_rotq; // d(acc)/d(rotq)
        matrix3f ang_acc_vel; // d(ang_acc)/d(vel)
        matrix3f ang_acc_ang_vel; // d(ang_acc)/d(ang_vel)
        matrixf<3, 4> ang_acc_rotq; // d(ang_acc)/d(rotq)
    };

public:
    /**
     * Calculate the model's expected acceleration in the global reference
     * frame based on the current state in [m/s^2]
     * @param linear_velocity Velocity in the global reference frame
     * @param rotation_q Rotation quaternion which transforms the local frame to global
     * @todo Can add typedef for vectors in the global and local reference frames
     */
    virtual vector3f get_linear_acceleration(
        const vector3f& linear_velocity,
        const quaternionf& rotation_q
    ) const noexcept = 0;

    /**
     * Calculate the model's expected angular acceleration based on the
     * current state in [rad/s^2]
     */
    virtual vector3f get_angular_acceleration(
        const vector3f& linear_velocity,
        const vector3f& angular_velocity,
        const quaternionf& rotation_q
    ) const noexcept = 0;

    /**
     * Derivative of components of `acc` and `ang_acc` vectors with respect to velocity,
     * rotation quaternion and angular velocity
     * @todo Pass `rotation_q` as quaternion
     */
    virtual jacobian_s get_jacobian(
        const vector3f& linear_velocity,
        const vector3f& angular_velocity,
        const vector4f& rotation_q
    ) const noexcept = 0;

    // /**
    //  * Process noise covariance matrix main diagonal
    //  * @note A diagonal matrix is created based on this vector
    //  */
    // virtual vector3f acc_cov_diagonal() const noexcept = 0;
    
    /**
     * Run an iteration of the control algorithm of the model
     * @note This method should control the actuators of the model
     */
    virtual void control(const state_s& state, float dt) noexcept = 0;
    
    /** @todo Add process_command(command_s& command) */

};

}