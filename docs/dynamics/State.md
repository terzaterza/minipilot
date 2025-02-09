# State
**State** is the collection of all variables which represent the current state of the system. This variables are continuously updated through the *state estimator task* and provided to the rest of the system for different purposes (telemetry fetches this state, packs it into protobuf messages and sends it, models use the current state to determine what actuator commands they need to send to achieve the user's commands, etc.).

## Notation
### Vectors
The symbol $\vec u$ represents a 3-dimensional vector called *u*, where $$ \vec u = \begin{bmatrix} u_{x} \\ u_y \\ u_z \end{bmatrix} $$
Norm of the vector $\vec u$ is the defined as $|\vec u| = \sqrt{u_x^2 + u_y^2 + u_z^2}$. Using the norm we define $u$'s unit vector as $\hat u = \frac{\vec u}{|\vec u|}$.

### Quaternions
#### Add quaternion definition and representation as a matrix

Rotation of a vector $\vec u$ by a quaternion $q$, written as ${\vec u}^q$, is defined by:
$$
{\vec u}^q = \Im \{ q \vec u q* \}
$$
where $\Im \{ q \}$ is the imaginary (vector) part of the quaternion.

## State variables
Model's **position** $p$ is the position of its center of mass ($p = p_{cm}$) in the *global (world) reference frame*, and is defined in units of meters, $[p] = \mathrm m$. Therefore its derivatives are: (linear) **velocity** $\vec v = \dot{\vec p}$ and (linear) **acceleration** $\vec a = \dot{\vec v} = \ddot{\vec p}$ with units of $\frac{\mathrm m}{\mathrm s}$ and $\frac{\mathrm m}{\mathrm s^2}$ respectively.

Origin of the global frame is defined at the system starting point, ie. when the system is turned on the position is set to $\vec p = \vec 0$.

Current **rotation** (**orientation**) of the model is described by a quaternion $q$ which maps the local reference frame to the global frame. Inverse mapping is done by taking the conjugate of this quaternion, for example $a^{q^*}$ represents the mapping of the acceleration vector to the local reference frame.

**Angular velocity vector** $\vec \omega$ is defined in the **local reference frame** in units of $\frac{\mathrm rad}{\mathrm s}$.