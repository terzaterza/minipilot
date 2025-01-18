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
    const auto v = get_linear_velocity(state);
    const auto a = get_linear_acceleration(state);
    const auto q = get_rotation_q(state);
    const auto w = get_angular_velocity(state);

    emblib::vector3f v_next = v + dt * a;
    emblib::vector3f a_next = m_model->get_linear_acceleration(v, q);

    // add if m_grounded and a_next.dot(DOWNWARD) >= 0 then v_next = a_next = 0
    // example for testing if grounded: true if vel downward ~ 0 and measured acc downward ~ 1g

    const float w1 = w(0);
    const float w2 = w(1);
    const float w3 = w(2);
    const emblib::matrixf<4> b {
        {0, -w1, -w2, -w3},
        {w1, 0, w3, -w2},
        {w2, -w3, 0, w1},
        {w3, w2, -w1, 0}
    };

    emblib::vectorf<4> qv = q.as_vector();
    emblib::vectorf<4> qv_next = qv + (dt / 2.f) * b.matmul(qv);
    // Normalize the quaternion due to numerical errors
    qv_next /= std::sqrt(qv.norm_sq());
    
    emblib::vector3f dw = m_model->get_angular_acceleration(v, w, q);
    emblib::vector3f w_next = w + dt * dw;

    return {
        v_next(0), v_next(1), v_next(2),
        a_next(0), a_next(1), a_next(2),
        qv_next(0), qv_next(1), qv_next(2), qv_next(3),
        w_next(0), w_next(1), w_next(2)
    };
}

emblib::matrixf<task_state_estimator::KALMAN_DIM>
task_state_estimator::state_transition_jacob(const state_vec_t& state) const noexcept
{
    emblib::matrixf<KALMAN_DIM> result {0};

    const auto v = get_linear_velocity(state);
    const auto a = get_linear_acceleration(state);
    const auto q = get_rotation_q(state);
    const auto w = get_angular_velocity(state);
    const auto qv = q.as_vector();

    // Not const because some matrices are multiplied by dt before
    // being inserted into the result matrix
    auto jacobian = m_model->get_jacobian(v, w, qv);

    // v_next = v + dt * a
    result(0, 0) = result(1, 1) = result(2, 2) = 1; // d(vel)/d(vel)
    result(0, 3) = result(1, 4) = result (2, 5) = dt; // d(vel)/d(acc)

    // a_next = f(v, q)
    set_submatrix<KALMAN_DIM, KALMAN_DIM, 3, 3>(result, 3, 0, jacobian.acc_vel); // d(acc)/d(vel)
    set_submatrix<KALMAN_DIM, KALMAN_DIM, 3, 4>(result, 3, 6, jacobian.acc_rotq); // d(acc)/d(rotq)

    // q_next = q + (dt/2) b(w)*q
    
    const float wx = w(0), wy = w(1), wz = w(2);
    const float qw = qv(0), qx = qv(1), qy = qv(2), qz = qv(3);
    
    const emblib::matrixf<4> q_q_jacob {
        {1, -dt/2*wx, -dt/2*wy, -dt/2*wz},
        {dt/2*wx, 1, dt/2*wz, -dt/2*wy},
        {dt/2*wy, -dt/2*wz, 1, dt/2*wx},
        {dt/2*wz, dt/2*wy, -dt/2*wx, 1}
    };

    const emblib::matrixf<4, 3> q_w_jacob = emblib::matrixf<4, 3> {
        {-qx, -qy, -qz},
        {qw, -qz, qy},
        {qz, qw, -qx},
        {-qy, qx, qw}
    } * (dt/2);

    set_submatrix<KALMAN_DIM, KALMAN_DIM, 4, 4>(result, 6, 6, q_q_jacob);
    set_submatrix<KALMAN_DIM, KALMAN_DIM, 4, 3>(result, 6, 10, q_w_jacob);

    // ang_vel_next = ang_vel_curr + dt * ang_acc(vel_curr, rotq_curr, ang_vel_curr)
    jacobian.ang_acc_vel *= dt;
    jacobian.ang_acc_rotq *= dt;
    jacobian.ang_acc_ang_vel *= dt;
    set_submatrix<KALMAN_DIM, KALMAN_DIM, 3, 3>(result, 10, 0, jacobian.ang_acc_vel); // d(ang_vel)/d(vel)
    set_submatrix<KALMAN_DIM, KALMAN_DIM, 3, 4>(result, 10, 6, jacobian.ang_acc_rotq); // d(ang_vel)/d(rotq)
    set_submatrix<KALMAN_DIM, KALMAN_DIM, 3, 3>(result, 10, 10, jacobian.ang_acc_ang_vel); // d(ang_vel)/d(ang_vel)
    result(10, 10) += 1.f;
    result(11, 11) += 1.f;
    result(12, 12) += 1.f;

    return result;
}

// This implementation assumes only 2 readings:
// acceleration and angular velocity
emblib::vectorf<task_state_estimator::OBS_DIM>
task_state_estimator::state_to_obs(const state_vec_t& state) const noexcept
{
    const auto a = get_linear_acceleration(state);
    const auto q = get_rotation_q(state);
    const auto w = get_angular_velocity(state);

    // Expected accelerometer reading = model acc + gravity mapped
    // to the local reference frame
    const emblib::vector3f a_exp = q.conjugate().rotate_vec(a + model::GRAVITY);

    // Expected gyroscope reading is just the angular velocity
    return {
        a_exp(0), a_exp(1), a_exp(2),
        w(0), w(1), w(2)
    };
}

// This implementation assumes only 2 readings:
// acceleration and angular velocity
emblib::matrixf<task_state_estimator::OBS_DIM, task_state_estimator::KALMAN_DIM>
task_state_estimator::state_to_obs_jacob(const state_vec_t& state) const noexcept
{
    emblib::matrixf<OBS_DIM, KALMAN_DIM> result {0};

    const auto a = get_linear_acceleration(state);
    const auto q = get_rotation_q(state);
    const auto w = get_angular_velocity(state);
    const auto qv = q.as_vector();

    const float ax = a(0), ay = a(1), az = a(2);
    const float qw = qv(0), qx = qv(1), qy = qv(2), qz = qv(3);
    constexpr float g = model::GRAVITY_CONST;
    
    // d(a_exp)/d(a)
    const emblib::matrixf<3> a_exp_a_jacob {
        {qw*qw + qx*qx - qy*qy - qz*qz, 2.f*(qw*qz + qx*qy), 2.f*(-qw*qy + qx*qz)},
        {2.f*(-qw*qz + qx*qy), qw*qw - qx*qx + qy*qy - qz*qz, 2.f*(qw*qx + qy*qz)},
        {2.f*(qw*qy + qx*qz), 2.f*(-qw*qx + qy*qz), qw*qw - qx*qx - qy*qy + qz*qz}
    };

    // d(a_exp)/d(qv)
    const emblib::matrixf<3, 4> a_exp_q_jacob {
        {2.f*(ax*qw + ay*qz - qy*(az - g)), 2.f*(ax*qx + ay*qy + qz*(az - g)), 2.f*(-ax*qy + ay*qx - qw*(az - g)), 2*(-ax*qz + ay*qw + qx*(az - g))},
        {2.f*(-ax*qz + ay*qw + qx*(az - g)), 2.f*(ax*qy - ay*qx + qw*(az - g)), 2.f*(ax*qx + ay*qy + qz*(az - g)), 2*(-ax*qw - ay*qz + qy*(az - g))},
        {2.f*(ax*qy - ay*qx + qw*(az - g)), 2.f*(ax*qz - ay*qw - qx*(az - g)), 2.f*(ax*qw + ay*qz - qy*(az - g)), 2*(ax*qx + ay*qy + qz*(az - g))}
    };

    set_submatrix<OBS_DIM, KALMAN_DIM, 3, 3>(result, 0, 3, a_exp_a_jacob);
    set_submatrix<OBS_DIM, KALMAN_DIM, 3, 4>(result, 0, 6, a_exp_q_jacob);

    // d(w_exp)/d(w)
    result(3, 10) = result(3, 11), result(3, 12) = 1.f;

    return result;
}

void task_state_estimator::run() noexcept
{
    while (true) {
        // Get latest sensor measurements
        auto a = m_task_accel.get_filtered();
        auto w = m_task_gyro.get_filtered();

        // If a measurement is not available, for example GPS, since it has a slower
        // update rate, create different observation vectors and different measurement
        // noise matrices for the kalman filter update

        const emblib::vectorf<6> observation {
            a(0), a(1), a(2), w(0), w(1), w(2)
        };

        // Build based on the model process noise
        const auto Q = emblib::vectorf<KALMAN_DIM>({
            .1f, .1f, .1f, .5f, .5f, .5f, .1f, .1f, .1f, .1f, .2f, .2f, .2f
        }).as_diagonal();

        // Get from the sensor tasks
        const auto R = emblib::vectorf<OBS_DIM>({
            .1f, .1f, .1f, .2f, .2f, .2f
        }).as_diagonal();

        m_kalman.update<OBS_DIM>(
            [this](const state_vec_t& state) {return state_transition(state);},
            [this](const state_vec_t& state) {return state_transition_jacob(state);},
            [this](const state_vec_t& state) {return state_to_obs(state);},
            [this](const state_vec_t& state) {return state_to_obs_jacob(state);},
            Q,
            R,
            observation
        );

        sleep_periodic(TASK_STATE_PERIOD);
    }
}

}