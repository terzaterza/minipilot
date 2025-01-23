import sympy

# Direction vectors
# Must match with model.hpp
FORWARD =   sympy.Matrix([1, 0, 0])
LEFT =      sympy.Matrix([0, 1, 0])
UP =        sympy.Matrix([0, 0, 1])
RIGHT =     -LEFT
DOWN =      -UP
BACK =      -FORWARD


# Gravity
g = sympy.symbols("g")
gv = DOWN * g

# Quaternion rotation
rot_v = lambda v, q: sympy.Matrix(sympy.Quaternion.rotate_point(v, q))


# State variables:

# Velocity vector (in the global frame)
vx, vy, vz = sympy.symbols("v_x v_y v_z")
v = sympy.Matrix([vx, vy, vz])

# Acceleration vector
ax, ay, az = sympy.symbols("a_x a_y a_z")
a = sympy.Matrix([ax, ay, az])

# Rotation quaternion mapping from local to the global frame
# Assuming that norm is always 1
qw, qx, qy, qz = sympy.symbols("q_w q_x q_y q_z")
q = sympy.Quaternion(qw, qx, qy, qz, norm=1)
qv = q.to_Matrix()

# Angular velocity
wx, wy, wz = sympy.symbols("omega_x omega_y omega_z")
w = sympy.Matrix([wx, wy, wz])