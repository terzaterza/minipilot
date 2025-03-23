#include "ekf_ahrs.hpp"
#include "mp/util/constants.hpp"

namespace mp {

ekf_ahrs::ekf_ahrs() noexcept :
    m_kalman({0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0})
{}

void
ekf_ahrs::update(const sensor_data_s& input, float dt) noexcept
{
    // TODO: Validate accel and gyro input not nullptr
    const vector3f a_in = *input.accelerometer;
    const vector3f w_in = *input.gyroscope;
    const vectorf<OBS_DIM> observation {
        a_in(0), a_in(1), a_in(2),
        w_in(0), w_in(1), w_in(2)
    };

    // Measurement (observation) variance
    matrixf<OBS_DIM> R(0);
    R.set_submatrix(0, 0, *input.accelerometer_cov);
    R.set_submatrix(3, 3, *input.gyroscope_cov);

    // TODO: Assign values using the kalman_state_e
    constexpr float a_noise = 5e-1;
    constexpr float q_noise = 1e-1;
    constexpr float w_noise = 5e-1;
    constexpr float wd_noise = 1e-1;
    const auto Q = vectorf<KALMAN_DIM>({
        a_noise, a_noise, a_noise,
        q_noise, q_noise, q_noise, q_noise,
        w_noise, w_noise, w_noise,
        wd_noise, wd_noise, wd_noise
    }).as_diagonal();

    // Run the kalman filter iteration
    m_kalman.update<OBS_DIM>(
        [this, &dt](const state_vec_t& state) {return state_transition(state, dt);},
        [this, &dt](const state_vec_t& state) {return state_transition_jacob(state, dt);},
        [this, &dt](const state_vec_t& state) {return state_to_obs(state, dt);},
        [this, &dt](const state_vec_t& state) {return state_to_obs_jacob(state, dt);},
        Q,
        R,
        observation
    );
}

// For implementation details view docs for this task
ekf_ahrs::state_vec_t
ekf_ahrs::state_transition(const state_vec_t& state, float dt) const noexcept
{
    const auto a = get_linear_acceleration(state);
    const auto q = get_rotation_q(state);
    const auto w = get_angular_velocity(state);
    const auto wd = get_gyro_drift(state);

    // Quaternion is updated according to the approximation of the first derivative of
    // the quaternion (w.r.t. time) as a function of angular velocity in the local frame
    const float w1 = w(0);
    const float w2 = w(1);
    const float w3 = w(2);
    const matrixf<4> b {
        {0, -w1, -w2, -w3},
        {w1, 0, w3, -w2},
        {w2, -w3, 0, w1},
        {w3, w2, -w1, 0}
    };
    
    vector4f qv = q.as_vector();
    vector4f qv_next = qv + (dt / 2.f) * b.matmul(qv);
    // Normalize the quaternion due to numerical errors
    qv_next /= qv_next.norm();
    
    return {
        a(0), a(1), a(2),
        qv_next(0), qv_next(1), qv_next(2), qv_next(3),
        w(0), w(1), w(2),
        wd(0), wd(1), wd(2)
    };
}

matrixf<ekf_ahrs::KALMAN_DIM>
ekf_ahrs::state_transition_jacob(const state_vec_t& state, float dt) const noexcept
{
    matrixf<KALMAN_DIM> result {0};

    const auto a = get_linear_acceleration(state);
    const auto q = get_rotation_q(state);
    const auto w = get_angular_velocity(state);
    const auto qv = q.as_vector();

    // da_da
    result(0, 0) = result(1, 1) = result(2, 2) = 1;
    
    // q_next = q + (dt/2) b(w)*q
    const float wx = w(0), wy = w(1), wz = w(2);
    const float qw = qv(0), qx = qv(1), qy = qv(2), qz = qv(3);
    const matrixf<4> dq_dq {
        {1, -dt/2*wx, -dt/2*wy, -dt/2*wz},
        {dt/2*wx, 1, dt/2*wz, -dt/2*wy},
        {dt/2*wy, -dt/2*wz, 1, dt/2*wx},
        {dt/2*wz, dt/2*wy, -dt/2*wx, 1}
    };
    const matrixf<4, 3> dq_dw = matrixf<4, 3> {
        {-qx, -qy, -qz},
        {qw, -qz, qy},
        {qz, qw, -qx},
        {-qy, qx, qw}
    } * (dt/2);
    result.set_submatrix(3, 3, dq_dq);
    result.set_submatrix(3, 7, dq_dw);
    
    // dw_dw
    // TODO: Add angular drag coefficient
    result(7, 7) = 1.f;
    result(8, 8) = 1.f;
    result(9, 9) = 1.f;

    // dwd_dwd
    result(10, 10) = 1.f;
    result(11, 11) = 1.f;
    result(12, 12) = 1.f;

    return result;
}

// This implementation assumes only 2 readings:
// acceleration and angular velocity
vectorf<ekf_ahrs::OBS_DIM>
ekf_ahrs::state_to_obs(const state_vec_t& state, float dt) const noexcept
{
    const auto a = get_linear_acceleration(state);
    const auto q = get_rotation_q(state);
    const auto w = get_angular_velocity(state);
    const auto wd = get_gyro_drift(state);

    // Expected accelerometer reading = model acc + gravity mapped
    // to the local reference frame
    const vector3f a_exp = q.conjugate().rotate_vec(a - GV);

    // Expected gyroscope reading = model ang vel + gyro drift
    const vector3f w_exp = w + wd;

    return {
        a_exp(0), a_exp(1), a_exp(2),
        w_exp(0), w_exp(1), w_exp(2)
    };
}

// This implementation assumes only 2 readings:
// acceleration and angular velocity
matrixf<ekf_ahrs::OBS_DIM, ekf_ahrs::KALMAN_DIM>
ekf_ahrs::state_to_obs_jacob(const state_vec_t& state, float dt) const noexcept
{
    matrixf<OBS_DIM, KALMAN_DIM> result {0};

    const auto a = get_linear_acceleration(state);
    const auto q = get_rotation_q(state);
    const auto qv = q.as_vector();

    const float ax = a(0), ay = a(1), az = a(2);
    const float qw = qv(0), qx = qv(1), qy = qv(2), qz = qv(3);
    
    // d(a_exp)/d(a)
    const matrixf<3> da_da {
        {qw*qw + qx*qx - qy*qy - qz*qz, 2.f*(qw*qz + qx*qy), 2.f*(-qw*qy + qx*qz)},
        {2.f*(-qw*qz + qx*qy), qw*qw - qx*qx + qy*qy - qz*qz, 2.f*(qw*qx + qy*qz)},
        {2.f*(qw*qy + qx*qz), 2.f*(-qw*qx + qy*qz), qw*qw - qx*qx - qy*qy + qz*qz}
    };

    // d(a_exp)/d(qv)
    const matrixf<3, 4> da_dq {
        {2.f*(ax*qw + ay*qz - qy*(az + G)), 2.f*(ax*qx + ay*qy + qz*(az + G)), 2.f*(-ax*qy + ay*qx - qw*(az + G)), 2*(-ax*qz + ay*qw + qx*(az + G))},
        {2.f*(-ax*qz + ay*qw + qx*(az + G)), 2.f*(ax*qy - ay*qx + qw*(az + G)), 2.f*(ax*qx + ay*qy + qz*(az + G)), 2*(-ax*qw - ay*qz + qy*(az + G))},
        {2.f*(ax*qy - ay*qx + qw*(az + G)), 2.f*(ax*qz - ay*qw - qx*(az + G)), 2.f*(ax*qw + ay*qz - qy*(az + G)), 2*(ax*qx + ay*qy + qz*(az + G))}
    };

    result.set_submatrix(0, 0, da_da);
    result.set_submatrix(0, 3, da_dq);

    // d(w_exp)/d(w)
    result(3, 7) = result(4, 8) = result(5, 9) = 1.f;

    // d(w_exp)/d(wd)
    result(3, 10) = result(4, 11) = result(5, 12) = 1.f;

    return result;
}
    
}