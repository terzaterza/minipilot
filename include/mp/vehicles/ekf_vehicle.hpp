#pragma once

#include "vehicle.hpp"

namespace mp {

/**
 * Adds required functionalities to a vehicle so it can be used
 * by the kalman filter
 */
class ekf_vehicle : public vehicle {

public:
    /**
     * Derivatives of linear and angular acceleration functions with
     * respect to state variables which might be used to calculate them
     * 
     * @note `dy_dx` means derivative (jacobian) of y with respect to x
     * @note `dw` is the angular acceleration (dw_dt), for others refer to
     * `state_s` documentation
     */
    struct jacobian_s {
        matrix3f da_dv;
        matrixf<3, 4> da_dq;
        matrix3f ddw_dv;
        matrix3f ddw_dw;
        matrixf<3, 4> ddw_dq;
    };

public:
    /**
     * Calculate the model's expected acceleration in the global (inertial) reference
     * frame based on the current state (velocity and rotation) in [m/s^2]
     * @param v Linear velocity in the global reference frame
     * @param q Rotation quaternion which transforms the local frame to global
     * @todo Can add typedef for vectors in the global and local reference frames
     */
    virtual vector3f get_linear_acceleration(
        const vector3f& v,
        const quaternionf& q
    ) const noexcept = 0;

    /**
     * Calculate the model's expected angular acceleration based on the
     * current state in [rad/s^2]
     * @param w Angular velocity in the body frame in [rad/s]
     */
    virtual vector3f get_angular_acceleration(
        const vector3f& v,
        const vector3f& w,
        const quaternionf& q
    ) const noexcept = 0;

    /**
     * Calculate the jacobian
     */
    virtual jacobian_s get_jacobian(
        const vector3f& v,
        const vector3f& w,
        const vector4f& qv
    ) const noexcept = 0;

    /// @todo Add get_process_noise() which returns the covariance matrix
};

}