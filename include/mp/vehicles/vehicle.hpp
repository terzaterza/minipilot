#pragma once

#include "mp/util/math.hpp"
#include "mp/vehicles/state.hpp"
#include "pb/command.pb.h"

namespace mp {

/**
 * Base class for all vehicles
 * 
 * Defines the interface via which the rest of the
 * system talks to different vehicles
 */
class vehicle {

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

    /**
     * Information about the sensor configurations onboard the vehicle
     * @todo Make the vehicle able to provide the list of all onboard
     * sensors and create the appropriate sensor task for each of those
     * and modify the state estimator to use all the appropriate sensors
     */
    struct sensor_config_s {
        // Map the accelerometer readings to the body coordinate frame
        matrix3f accelerometer_transform;
        // Map the gyroscope readings to the body coordinate frame
        matrix3f gyroscope_transform;
        
        // vector3f accelerometer_com_displacement;
    };

public:
    /**
     * Calculate the model's expected acceleration in the global (inertial) reference
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
     * Calculate the jacobian
     */
    virtual jacobian_s get_jacobian(
        const vector3f& linear_velocity,
        const vector3f& angular_velocity,
        const vector4f& rotation_qv
    ) const noexcept = 0;

    /**
     * Runs once at the beginning of the vehicle task
     * @todo Provide the state as a parameter
     */
    virtual void init() noexcept = 0;

    /**
     * Update the internal state (control algorithm for example)
     */
    virtual void update(const state_s& state, float dt) noexcept = 0;

    /**
     * @returns false if the command is not for this vehicle type
     */
    virtual bool handle_command(const pb::Command& command) noexcept = 0;

    /**
     * Get information about onboard sensors
     * TODO: View `sensor_config_s` description
     */
    virtual const sensor_config_s& get_sensor_config() const noexcept = 0;
    
    /// @todo Add get_cov_matrix()

};

}