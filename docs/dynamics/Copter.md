# Copter
A **copter** is a vehicle which produces thrust in a single direction (up), and can produce torque in any direction independently.

## Parameters
- $m$ - Mass
- $c_d$ - Linear drag coefficient
- $I$ - Inertia matrix

## Dynamics
**Linear acceleration** $\vec a$ of a copter uses the following model:
$$ m\vec{a} = m\vec{g} + \vec{T}^q - c_d \vec v $$
where $\vec{g}$ represents the acceleration due to gravity, $\vec{T} = T \hat u_{up}$ with $T$ being the amount of thrust produced (in Newtons) and $\vec u_{up}$ is the unit vector in the up direction. Since $\vec T$ is thrust procuded upwards in the local reference frame, it is mapped to the global frame via $q$. Drag force is assumed to be linear (should be changed to quadratic), $F_d = -c_d \vec v$.

**Angular acceleration** $\vec \alpha = \dot{\vec \omega}$ is based on the equations for angular momentum in a rotating reference frame:
$$ I \vec \alpha = \vec \tau - \vec \omega \times (I \vec \omega) $$
where $\tau$ is the torque produced by the vehicle, and all vectors are defined in the reference frame attached to the vehicle.

## Control system
There are two modes of controlling a copter:
- Angular velocity
- Linear velocity

### Angular velocity control
Angular velocity controller is a part (subsystem) of the copter control system responsible for achieving the angular velocity vector set by the user or another part of the control system. From the angular acceleration model, we can express the torque $\tau$ as a function of the angular velocity and acceleration:
$$
\tau(\alpha, \omega) = I \alpha + \omega \times (I \omega)
$$
For some target acceleration $\alpha_t$ and the current angular velocity $\omega$, we get the torque needed to produce that acceleration. The target acceleration can be computed using a PID controller based on the difference of the current angular velocity and the desired one:
$$
\alpha_t = (\omega_t - \omega)F_\omega(s)
$$
where $F_\omega(s) = k_p + \frac{k_i}{s} + k_d s$ is the transfer function of the PID controller. Constants in the PID are used to control how fast the model achieves the desired angular velocity and the stability of this subsystem.

Since a copter produces torque independent of thrust, using $\alpha_t$ and some target thrust $T_t$, the inverse motor mixing algorithm (MMA) can compute the needed motor speeds to produce this torque and thrust.

### Linear velocity control
A copter has 4 degrees of freedom, 3 from torque and 1 from thrust generation. Therefore velocity is changed by first rotating the copter so that the thrust direction points towards the direction of the target velocity, but also compensating for the force of gravity acting on the object (if the object is not grounded) and other possible external forces (wind, etc.).

Similarly to angular velocity control, from the linear motion equation we can express the required thrust vector to achieve some acceleration $a_t$:
$$
\vec T_{tg}(\vec{a_t}) = m \vec{a_t} - m \vec g + c_d \vec v
$$
where $\vec T_{tg}$ is the thrust vector in the global reference frame. Now that we have a thrust vector, we need to rotate the copter so that its thrust generation direction is aligned with this vector. To do this we need to produce the angular velocity required for this rotation and pass it to the previously described angular velocity control subsystem.

Target thrust vector in the local frame is $\vec T_t = {\vec T_{tg}}^{q^*}$. Direction (ignoring the magnitude) of the angular velocity vector required to align these vectors is the cross product
$$
\vec \omega_{dir} = \hat u_{up} \times \hat T_t
$$
Since both of the terms in the cross product are normalized, magnitude of $\vec w_{dir}$ is between 0 and 1. We can multiply this vector by a constant to determine with what angular speed do we achieve the desired orientation without introducing instability.