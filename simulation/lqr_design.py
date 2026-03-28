"""
LQR controller design for the inverted pendulum reaction motor.

The reaction motor ONLY controls the pendulum angle — the cart is driven
separately by the wheel motors.  We therefore design a 2-state LQR:

  State  : [theta, theta_dot]   (angle from vertical, angular velocity)
  Input  : tau                  (reaction motor torque, N·m)
  Dist.  : x_ddot_cart          (cart acceleration — feedforward if needed)

Linearised around the upright equilibrium (theta = 0).

Note on the 4-state full cart-pendulum model:
  A 4-state CARE would fail because cart position x is a pure integrator
  that creates imaginary-axis poles in the Hamiltonian.  The 2-state
  formulation avoids this and matches the physical role of the reaction motor.
"""

import numpy as np
from scipy.linalg import solve_continuous_are

# ── Physical parameters (must match pendulum_sim.py) ────────────────────────
PEND_MASS    = 0.3          # m  (kg)
PEND_LENGTH  = 0.5          # L  (m)   — full length of uniform rod
GRAVITY      = 9.81         # g  (m/s²)
PEND_FRIC    = 0.01         # b_pend (N·m·s/rad)

# Moment of inertia of uniform rod about hinge (end)
I_HINGE = (1.0 / 3.0) * PEND_MASS * PEND_LENGTH ** 2


def get_system_matrices():
    """
    Return 2-state linearised matrices (A, B) for the pendulum subsystem.

    From the rotational EOM (decoupled, cart acceleration treated as disturbance):
      I_h * theta_ddot = (m*g*L/2)*theta - b_p*theta_dot + tau

    State: [theta, theta_dot]    Input: tau (N·m)
    """
    m, L, g = PEND_MASS, PEND_LENGTH, GRAVITY
    b_p     = PEND_FRIC
    I_h     = I_HINGE

    A = np.array([
        [0.0,                      1.0   ],
        [m * g * L / (2.0 * I_h), -b_p / I_h],
    ])

    B = np.array([[0.0], [1.0 / I_h]])

    return A, B


def compute_lqr_gains(Q=None, R=None):
    """
    Solve the continuous-time LQR and return gain vector K (shape (2,)).

    Control law:  tau = -K @ [theta, theta_dot]

    Default weights
    ---------------
    Q  — penalises angle heavily, angular velocity moderately.
    R  — low torque penalty so the controller reacts quickly.

    Returns
    -------
    K : ndarray, shape (2,)
    """
    A, B = get_system_matrices()

    if Q is None:
        Q = np.diag([120.0, 20.0])
    if R is None:
        R = np.array([[0.01]])

    P = solve_continuous_are(A, B, Q, R)
    K = (np.linalg.inv(R) @ B.T @ P).flatten()
    return K


def print_system_info(Q=None, R=None):
    """Print A, B, eigenvalues and closed-loop stability summary."""
    A, B = get_system_matrices()
    K    = compute_lqr_gains(Q, R)

    ol_eigs = np.linalg.eigvals(A)
    A_cl    = A - B @ K.reshape(1, -1)
    cl_eigs = np.linalg.eigvals(A_cl)
    stable  = all(e.real < 0 for e in cl_eigs)

    print("=" * 50)
    print("  Pendulum Subsystem State-Space (2-state, linearised)")
    print("=" * 50)
    print(f"\nA =\n{np.array2string(A, precision=4)}")
    print(f"\nB =\n{np.array2string(B, precision=4)}")
    print(f"\nOpen-loop eigenvalues : {ol_eigs.real.round(3)}")
    print(f"LQR gains K           : {K.round(4)}")
    print(f"Closed-loop eigenvalues: {cl_eigs.round(3)}")
    print(f"Closed-loop stable     : {stable}")
    print("=" * 50)


if __name__ == "__main__":
    print_system_info()
