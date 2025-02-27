#include "ekf_inertial.hpp"
#include "mp/util/constants.hpp"

namespace mp {

ekf_inertial::ekf_inertial(const ekf_vehicle& vehicle) noexcept :
    m_vehicle(vehicle),
    m_kalman({0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0})
{}

void
ekf_inertial::update(const sensor_data_s& input, float dt) noexcept
{
    // Information about sensor placement on the vehicle
    // TODO: Can be moved to `state_to_obs`, but inverse mappings will be needed
    const vehicle::sensor_config_s& sensor_config = m_vehicle.get_sensor_config();

    // TODO: Validate accel and gyro input not nullptr
    const vector3f a_in = sensor_config.accelerometer_transform.matmul(*input.accelerometer);
    const vector3f w_in = sensor_config.gyroscope_transform.matmul(*input.gyroscope);
    const vectorf<OBS_DIM> observation {
        a_in(0), a_in(1), a_in(2),
        w_in(0), w_in(1), w_in(2)
    };

    // Measurement (observation) variance
    matrixf<OBS_DIM> R(0);
    R.set_submatrix(0, 0, *input.accelerometer_cov);
    R.set_submatrix(3, 3, *input.gyroscope_cov);

    // TODO: Get Q from the vehicle
    constexpr float v_noise = 1;
    constexpr float a_noise = 5e-1;
    constexpr float q_noise = 1e-1;
    constexpr float w_noise = 5e-1;
    const auto Q = vectorf<KALMAN_DIM>({
        v_noise, v_noise, v_noise,
        a_noise, a_noise, a_noise,
        q_noise, q_noise, q_noise, q_noise,
        w_noise, w_noise, w_noise
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

    // Position is integration of velocity and acceleration
    const auto v = get_linear_velocity(m_kalman.get_state());
    const auto a = get_linear_acceleration(m_kalman.get_state());
    m_position += v * dt + a * (dt * dt / 2.f);
}

// For implementation details view docs for this task
ekf_inertial::state_vec_t
ekf_inertial::state_transition(const state_vec_t& state, float dt) const noexcept
{
    const auto v = get_linear_velocity(state);
    const auto a = get_linear_acceleration(state);
    const auto q = get_rotation_q(state);
    const auto w = get_angular_velocity(state);

    // Acceleration is computed by the vehicle based on current actuator settings and the
    // dynamical model of the vehicle, and the velocity is the integration of acceleration
    vector3f v_next = v + dt * a;
    vector3f a_next = m_vehicle.get_linear_acceleration(v, q);

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
    
    // Angular acceleration is the first derivative of angular velocity and
    // is calculated according to the Euler's equations for a rotating reference frame
    vector3f dw = m_vehicle.get_angular_acceleration(v, w, q);
    vector3f w_next = w + dt * dw;

    return {
        v_next(0), v_next(1), v_next(2),
        a_next(0), a_next(1), a_next(2),
        qv_next(0), qv_next(1), qv_next(2), qv_next(3),
        w_next(0), w_next(1), w_next(2)
    };
}

matrixf<ekf_inertial::KALMAN_DIM>
ekf_inertial::state_transition_jacob(const state_vec_t& state, float dt) const noexcept
{
    matrixf<KALMAN_DIM> result {0};

    const auto v = get_linear_velocity(state);
    const auto a = get_linear_acceleration(state);
    const auto q = get_rotation_q(state);
    const auto w = get_angular_velocity(state);
    const auto qv = q.as_vector();

    // Not const because some matrices are multiplied by dt before
    // being inserted into the result matrix
    auto jacobian = m_vehicle.get_jacobian(v, w, qv);

    // v_next = v + dt * a
    result(0, 0) = result(1, 1) = result(2, 2) = 1; // dv_dv
    result(0, 3) = result(1, 4) = result (2, 5) = dt; // dv_da

    // a_next = f(v, q)
    result.set_submatrix(3, 0, jacobian.da_dv);
    result.set_submatrix(3, 6, jacobian.da_dq);
    
    const float wx = w(0), wy = w(1), wz = w(2);
    const float qw = qv(0), qx = qv(1), qy = qv(2), qz = qv(3);
    
    // q_next = q + (dt/2) b(w)*q
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
vectorf<ekf_inertial::OBS_DIM>
ekf_inertial::state_to_obs(const state_vec_t& state, float dt) const noexcept
{
    const auto a = get_linear_acceleration(state);
    const auto q = get_rotation_q(state);
    const auto w = get_angular_velocity(state);

    // Expected accelerometer reading = model acc + gravity mapped
    // to the local reference frame
    const vector3f a_exp = q.conjugate().rotate_vec(a - GV);

    // Expected gyroscope reading is just the angular velocity
    return {
        a_exp(0), a_exp(1), a_exp(2),
        w(0), w(1), w(2)
    };
}

// This implementation assumes only 2 readings:
// acceleration and angular velocity
matrixf<ekf_inertial::OBS_DIM, ekf_inertial::KALMAN_DIM>
ekf_inertial::state_to_obs_jacob(const state_vec_t& state, float dt) const noexcept
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

    result.set_submatrix(0, 3, da_da);
    result.set_submatrix(0, 6, da_dq);

    // d(w_exp)/d(w)
    result(3, 10) = result(4, 11) = result(5, 12) = 1.f;

    return result;
}
    
}