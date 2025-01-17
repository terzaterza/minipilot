#pragma once

#include "state.hpp"
#include "util/actuators.hpp"
#include "emblib/math/vector.hpp"

namespace mp {

/**
 * Model of the vehicle with respect to dynamics
 */
class model {

public:
    // Strucutre contatining all derivatives of `acc` and `ang_acc` with respect to state
    struct jacobian_s {
        emblib::matrixf<3> acc_vel; // d(acc)/d(vel)
        emblib::matrixf<3, 4> acc_rotq; // d(acc)/d(rotq)
        emblib::matrixf<3> ang_acc_vel; // d(ang_acc)/d(vel)
        emblib::matrixf<3, 4> ang_acc_rotq; // d(ang_acc)/d(rotq)
        emblib::matrixf<3> ang_acc_ang_vel; // d(ang_acc)/d(ang_vel)
    };

    // Model direction definitions
    static inline const emblib::vector3f FORWARD    = {1, 0, 0};
    static inline const emblib::vector3f LEFT       = {0, 1, 0};
    static inline const emblib::vector3f UP         = {0, 0, 1};
    static inline const emblib::vector3f BACKWARD   = -FORWARD;
    static inline const emblib::vector3f RIGHT      = -LEFT;
    static inline const emblib::vector3f DOWN       = -UP;

    // Gravity vector definition
    static inline const emblib::vector3f GRAVITY    = DOWN * 9.80655f;

public:
    /**
     * Calculate the model's expected acceleration in the global reference
     * frame based on the current state in [m/s^2]
     * @param vel Velocity in the global reference frame
     * @param rotq Rotation quaternion which transforms the local frame to global
     * @todo Can add typedef for vectors in the global and local reference frames
     */
    virtual emblib::vector3f acc(
        const emblib::vector3f& vel,
        const emblib::quaternionf& rotq
    ) const noexcept = 0;

    /**
     * Calculate the model's expected angular acceleration based on the
     * current state in [rad/s^2]
     */
    virtual emblib::vector3f ang_acc(
        const emblib::vector3f& vel,
        const emblib::quaternionf& rotq,
        const emblib::vector3f& ang_vel
    ) const noexcept = 0;

    /**
     * Derivative of components of `acc` and `ang_acc` vectors with respect to velocity,
     * rotation quaternion and angular velocity
     */
    virtual jacobian_s jacobian(
        const emblib::vector3f& vel,
        const emblib::vectorf<4>& rotqv,
        const emblib::vector3f& ang_vel
    ) const noexcept = 0;
    
    /**
     * Run an iteration of the control algorithm of the model
     * @note This method should control the actuators of the model
     */
    virtual void control(const state_s& state, const actuators_s& actuators) noexcept = 0;

    
    /** @todo Add process_command(command_s& command) */

};

}