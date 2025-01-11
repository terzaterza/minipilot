#include "task_state_estimator.hpp"
#include "util/logger.hpp"
#include <cmath>

namespace mp {

task_state_estimator::state_vec_t task_state_estimator::state_transition(const state_vec_t& state) const noexcept
{
    constexpr std::chrono::duration<float> PERIOD_SECONDS = TASK_STATE_PERIOD;
    constexpr float dt = PERIOD_SECONDS.count();

    state_s current_state {
        .position = m_position,
        .velocity = get_vel(state),
        .acceleration = get_acc(state),
        .ang_velocity = get_ang_vel(state),
        .rotationq = get_rotq(state)
    };

    emblib::vector3f vel_next = current_state.velocity + dt * current_state.acceleration;
    emblib::vector3f acc_next = m_model->acc(current_state);

    // add if m_grounded and acc_next.dot(DOWNWARD) >= 0 then vel_next = acc_next = 0
    // example for testing if grounded: true if vel downward ~ 0 and measured acc downward ~ 1g

    const float w1 = current_state.ang_velocity(0);
    const float w2 = current_state.ang_velocity(1);
    const float w3 = current_state.ang_velocity(2);
    const emblib::matrixf<4> b {
        {0, -w1, -w2, -w3},
        {w1, 0, w3, -w2},
        {w2, -w3, 0, w1},
        {w3, w2, -w1, 0}
    };


    emblib::vectorf<4> rotq_v = current_state.rotationq.as_vector();
    emblib::vectorf<4> rotq_v_next = rotq_v + (dt / 2.f) * b.matmul(rotq_v);
    rotq_v_next /= std::sqrt(rotq_v_next.norm_sq());
    
    emblib::vector3f ang_acc = m_model->ang_acc(current_state);
    emblib::vector3f ang_vel_next = current_state.ang_velocity + dt * ang_acc;

    return {
        vel_next(0), vel_next(1), vel_next(2),
        acc_next(0), acc_next(1), acc_next(2),
        rotq_v_next(0), rotq_v_next(1), rotq_v_next(2), rotq_v_next(3),
        ang_vel_next(0), ang_vel_next(1), ang_vel_next(2)
    };
}

void task_state_estimator::run() noexcept
{
    while (true) {
        auto accel = m_task_accel.get_filtered();
        auto ang_vel = m_task_gyro.get_filtered();



        sleep_periodic(TASK_STATE_PERIOD);
    }
}

}