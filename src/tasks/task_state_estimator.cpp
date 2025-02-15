#include "mp/util/constants.hpp"
#include "task_state_estimator.hpp"
#include "util/logger.hpp"
#include <cmath>

namespace mp {

// Conversion of the task period to floating point delta time
static constexpr float dt = std::chrono::duration<float>(TASK_STATE_PERIOD).count();
// Max deviation if we are assuming being grounded
static constexpr float GROUNDED_VERTICAL_VELOCITY_ERROR = 0.01;


// For implementation details view docs for this task
task_state_estimator::state_vec_t
task_state_estimator::state_transition(const state_vec_t& state) const noexcept
{
    const auto v = get_linear_velocity(state);
    const auto a = get_linear_acceleration(state);
    const auto q = get_rotation_q(state);
    const auto w = get_angular_velocity(state);

    emblib::vector3f v_next = v + dt * a;
    emblib::vector3f a_next = m_vehicle->get_linear_acceleration(v, q);

    // If we are grounded and the expected acceleration is downwards (for example if the quadcopter
    // is producing less than hover lift), remove the downwards acceleration and velocity components
    const float a_down = a_next.dot(DOWN);
    if (m_grounded && a_down >= 0) {
        const emblib::vector3f down_mask = (!DOWN).cast<float>();
        a_next *= down_mask;
        v_next *= down_mask;
    }

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
    
    emblib::vector3f dw = m_vehicle->get_angular_acceleration(v, w, q);
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
    auto jacobian = m_vehicle->get_jacobian(v, w, qv);

    // v_next = v + dt * a
    result(0, 0) = result(1, 1) = result(2, 2) = 1; // dv_dv
    result(0, 3) = result(1, 4) = result (2, 5) = dt; // dv_da

    // a_next = f(v, q)
    result.set_submatrix(3, 0, jacobian.da_dv);
    result.set_submatrix(3, 6, jacobian.da_dq);
    
    const float wx = w(0), wy = w(1), wz = w(2);
    const float qw = qv(0), qx = qv(1), qy = qv(2), qz = qv(3);
    
    // q_next = q + (dt/2) b(w)*q
    const emblib::matrixf<4> dq_dq {
        {1, -dt/2*wx, -dt/2*wy, -dt/2*wz},
        {dt/2*wx, 1, dt/2*wz, -dt/2*wy},
        {dt/2*wy, -dt/2*wz, 1, dt/2*wx},
        {dt/2*wz, dt/2*wy, -dt/2*wx, 1}
    };
    const emblib::matrixf<4, 3> dq_dw = emblib::matrixf<4, 3> {
        {-qx, -qy, -qz},
        {qw, -qz, qy},
        {qz, qw, -qx},
        {-qy, qx, qw}
    } * (dt/2);

    result.set_submatrix(6, 6, dq_dq);
    result.set_submatrix(6, 10, dq_dw);

    // w_next = w + dt * dw(v, q, w)
    jacobian.ddw_dv *= dt;
    jacobian.ddw_dq *= dt;
    jacobian.ddw_dw *= dt;
    result.set_submatrix(10, 0, jacobian.ddw_dv);
    result.set_submatrix(10, 6, jacobian.ddw_dq);
    result.set_submatrix(10, 10, jacobian.ddw_dw);
    
    // dw_dw
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
    const emblib::vector3f a_exp = q.conjugate().rotate_vec(a + GV);

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
    
    // d(a_exp)/d(a)
    const emblib::matrixf<3> a_exp_a_jacob {
        {qw*qw + qx*qx - qy*qy - qz*qz, 2.f*(qw*qz + qx*qy), 2.f*(-qw*qy + qx*qz)},
        {2.f*(-qw*qz + qx*qy), qw*qw - qx*qx + qy*qy - qz*qz, 2.f*(qw*qx + qy*qz)},
        {2.f*(qw*qy + qx*qz), 2.f*(-qw*qx + qy*qz), qw*qw - qx*qx - qy*qy + qz*qz}
    };

    // d(a_exp)/d(qv)
    const emblib::matrixf<3, 4> a_exp_q_jacob {
        {2.f*(ax*qw + ay*qz - qy*(az - G)), 2.f*(ax*qx + ay*qy + qz*(az - G)), 2.f*(-ax*qy + ay*qx - qw*(az - G)), 2*(-ax*qz + ay*qw + qx*(az - G))},
        {2.f*(-ax*qz + ay*qw + qx*(az - G)), 2.f*(ax*qy - ay*qx + qw*(az - G)), 2.f*(ax*qx + ay*qy + qz*(az - G)), 2*(-ax*qw - ay*qz + qy*(az - G))},
        {2.f*(ax*qy - ay*qx + qw*(az - G)), 2.f*(ax*qz - ay*qw - qx*(az - G)), 2.f*(ax*qw + ay*qz - qy*(az - G)), 2*(ax*qx + ay*qy + qz*(az - G))}
    };

    result.set_submatrix(0, 3, a_exp_a_jacob);
    result.set_submatrix(0, 6, a_exp_q_jacob);

    // d(w_exp)/d(w)
    result(3, 10) = result(3, 11), result(3, 12) = 1.f;

    return result;
}

void task_state_estimator::run() noexcept
{
    while (true) {
        // Get latest sensor measurements
        auto a_read = m_task_accel.get_filtered();
        auto w_read = m_task_gyro.get_filtered();

        // If a measurement is not available, for example GPS, since it has a slower
        // update rate, create different observation vectors and different measurement
        // noise matrices for the kalman filter update

        const emblib::vectorf<6> observation {
            a_read(0), a_read(1), a_read(2), w_read(0), w_read(1), w_read(2)
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

        const auto v = get_linear_velocity(m_kalman.get_state());
        const auto a = get_linear_acceleration(m_kalman.get_state());
        const auto q = get_rotation_q(m_kalman.get_state());
        m_position += v * dt + a * (dt * dt / 2.f);

        // Check if we're grounded based on the expected acceleration and the actual one
        const bool vertical_vel_zero = std::abs(v.dot(UP)) < GROUNDED_VERTICAL_VELOCITY_ERROR;
        const bool insufficient_acc = m_vehicle->get_linear_acceleration(v, q).dot(UP) < 0;
        m_grounded = vertical_vel_zero && insufficient_acc;

        // Assign the kalman state to the readable state struct
        m_state_mutex.lock();
        m_state.position = m_position;
        m_state.velocity = v;
        m_state.acceleration = a;
        m_state.rotationq = q;
        m_state.angular_velocity = get_angular_velocity(m_kalman.get_state());
        m_state.grounded = m_grounded;
        m_state_mutex.unlock();

        sleep_periodic(TASK_STATE_PERIOD);
    }
}

}