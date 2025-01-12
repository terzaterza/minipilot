#include "task_state_estimator.hpp"
#include "util/logger.hpp"
#include <cmath>

namespace mp {

constexpr std::chrono::duration<float> PERIOD_SECONDS = TASK_STATE_PERIOD;
constexpr float dt = PERIOD_SECONDS.count();

// Submatrix assignment
template <size_t R1, size_t C1, size_t R2, size_t C2>
static void set_submatrix(
    emblib::matrixf<R1, C1>& dest,
    size_t top,
    size_t left,
    const emblib::matrixf<R2, C2>& src)
{
    for (size_t r = 0; r < R2; r++) {
        for (size_t c = 0; c < C2; c++) {
            dest(top + r, left + c) = src(r, c);
        }
    }
}

// For implementation details view docs for this task
task_state_estimator::state_vec_t
task_state_estimator::state_transition(const state_vec_t& state) const noexcept
{
    const auto vel_curr = get_vel(state);
    const auto acc_curr = get_acc(state);
    const auto rotq_curr = get_rotq(state);
    const auto ang_vel_curr = get_ang_vel(state);

    emblib::vector3f vel_next = vel_curr + dt * acc_curr;
    emblib::vector3f acc_next = m_model->acc(vel_curr, rotq_curr);

    // add if m_grounded and acc_next.dot(DOWNWARD) >= 0 then vel_next = acc_next = 0
    // example for testing if grounded: true if vel downward ~ 0 and measured acc downward ~ 1g

    const float w1 = ang_vel_curr(0);
    const float w2 = ang_vel_curr(1);
    const float w3 = ang_vel_curr(2);
    const emblib::matrixf<4> b {
        {0, -w1, -w2, -w3},
        {w1, 0, w3, -w2},
        {w2, -w3, 0, w1},
        {w3, w2, -w1, 0}
    };

    emblib::vectorf<4> rotq_v = rotq_curr.as_vector();
    emblib::vectorf<4> rotq_v_next = rotq_v + (dt / 2.f) * b.matmul(rotq_v);
    rotq_v_next /= std::sqrt(rotq_v_next.norm_sq());
    
    emblib::vector3f ang_acc = m_model->ang_acc(vel_curr, rotq_curr, ang_vel_curr);
    emblib::vector3f ang_vel_next = ang_vel_curr + dt * ang_acc;

    return {
        vel_next(0), vel_next(1), vel_next(2),
        acc_next(0), acc_next(1), acc_next(2),
        rotq_v_next(0), rotq_v_next(1), rotq_v_next(2), rotq_v_next(3),
        ang_vel_next(0), ang_vel_next(1), ang_vel_next(2)
    };
}

emblib::matrixf<task_state_estimator::KALMAN_DIM>
task_state_estimator::state_transition_jacob(const state_vec_t& state) const noexcept
{
    const auto vel_curr = get_vel(state);
    const auto acc_curr = get_acc(state);
    const auto rotq_curr = get_rotq(state);
    const auto ang_vel_curr = get_ang_vel(state);
    const auto rotq_curr_v = rotq_curr.as_vector();

    const auto acc_vel_jacob = m_model->acc_vel_jacobian(vel_curr);
    const auto acc_rotq_jacob = m_model->acc_rotq_jacobian(rotq_curr_v);
    auto ang_acc_vel_jacob = m_model->ang_acc_vel_jacobian(vel_curr);
    auto ang_acc_rotq_jacob = m_model->ang_acc_rotq_jacobian(rotq_curr_v);
    auto ang_acc_ang_vel_jacob = m_model->ang_acc_ang_vel_jacobian(ang_vel_curr);

    const float w1 = ang_vel_curr(0);
    const float w2 = ang_vel_curr(1);
    const float w3 = ang_vel_curr(2);

    const float q1 = rotq_curr_v(0);
    const float q2 = rotq_curr_v(0);
    const float q3 = rotq_curr_v(0);
    const float q4 = rotq_curr_v(0);

    emblib::matrixf<KALMAN_DIM> result {0};

    // vel_next = vel_curr + dt * acc_curr
    result(0, 0) = result(1, 1) = result(2, 2) = 1; // d(vel)/d(vel)
    result(0, 3) = result(1, 4) = result (2, 5) = dt; // d(vel)/d(acc)

    // acc_next = f(vel_curr, rotq_curr)
    set_submatrix<KALMAN_DIM, KALMAN_DIM, 3, 3>(result, 3, 0, acc_vel_jacob); // d(acc)/d(vel)
    set_submatrix<KALMAN_DIM, KALMAN_DIM, 3, 4>(result, 3, 6, acc_rotq_jacob); // d(acc)/d(rotq)

    // rotq_next = rotq_curr + (dt/2) b(ang_vel) * rotq_curr
    result(6, 6) = result(7, 7) = result(8, 8) = result(9, 9) = 1; // d(rotq)/d(rotq)
    result(6, 7) = -dt / 2 * w1;
    result(6, 8) = -dt / 2 * w2;
    result(6, 9) = -dt / 2 * w3;
    result(7, 6) = dt / 2 * w1;
    result(7, 8) = dt / 2 * w3;
    result(7, 9) = -dt / 2 * w2;
    result(8, 6) = dt / 2 * w2;
    result(8, 7) = -dt / 2 * w3;
    result(8, 9) = dt / 2 * w1;
    result(9, 6) = dt / 2 * w3;
    result(9, 7) = dt / 2 * w2;
    result(9, 8) = -dt / 2 * -w1;
    result(6, 10) = -dt / 2 * q2; // d(rotq)/d(ang_vel)
    result(6, 11) = -dt / 2 * q3;
    result(6, 12) = -dt / 2 * q4;
    result(7, 10) = dt / 2 * q1;
    result(7, 11) = -dt / 2 * q4;
    result(7, 12) = dt / 2 * q3;
    result(8, 10) = dt / 2 * q4;
    result(8, 10) = dt / 2 * q1;
    result(8, 10) = -dt / 2 * q2;
    result(9, 10) = -dt / 2 * q2;
    result(9, 11) = -dt / 2 * q2;
    result(9, 12) = -dt / 2 * q2;

    // ang_vel_next = ang_vel_curr + dt * ang_acc(vel_curr, rotq_curr, ang_vel_curr)
    ang_acc_vel_jacob *= dt;
    ang_acc_rotq_jacob *= dt;
    ang_acc_ang_vel_jacob *= dt;
    set_submatrix<KALMAN_DIM, KALMAN_DIM, 3, 3>(result, 10, 0, ang_acc_vel_jacob); // d(ang_vel)/d(vel)
    set_submatrix<KALMAN_DIM, KALMAN_DIM, 3, 4>(result, 10, 6, ang_acc_rotq_jacob); // d(ang_vel)/d(rotq)
    set_submatrix<KALMAN_DIM, KALMAN_DIM, 3, 3>(result, 10, 10, ang_acc_ang_vel_jacob); // d(ang_vel)/d(ang_vel)
    result(10, 10) += 1.f;
    result(11, 11) += 1.f;
    result(12, 12) += 1.f;

    return result;
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