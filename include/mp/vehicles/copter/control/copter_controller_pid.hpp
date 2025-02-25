#pragma once

#include "mp/vehicles/copter/control/copter_controller.hpp"
#include "mp/vehicles/copter/copter.hpp"
#include "emblib/dsp/pid.hpp"

namespace mp {

class copter_controller_pid : public copter_controller {

enum class control_mode_e {
    ANGULAR,
    LINEAR
};

public:
    copter_controller_pid(const copter_params_s& copter_params) noexcept;

    bool set_target_w(const vector3f& target_w, float target_thrust) noexcept override;

    bool set_target_v(const vector3f& target_v, float target_dir) noexcept override;
    
    void update(const ins_state_s& state, float dt) noexcept override;
    
    vector3f get_torque() const noexcept override
    {
        return m_output_torque;
    }

    float get_thrust() const noexcept override
    {
        return m_output_thrust;
    }

private:
    const copter_params_s& m_copter_params;

    control_mode_e m_control_mode;
    vector3f m_target_w;
    vector3f m_target_v;
    float m_target_dir;

    vector3f m_output_torque;
    float m_output_thrust;

    emblib::pid<vector3f, float> m_angular_velocity_pid;
    emblib::pid<vector3f, float> m_linear_acceleration_pid;
};

}