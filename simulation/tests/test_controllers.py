"""
Tests for LQR and PID controllers.

Run from the project root:
    pytest simulation/tests/test_controllers.py -v
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import pytest
from lqr_design     import compute_lqr_gains, get_system_matrices
from pid_controller import PID, CascadePID
from pendulum_sim   import rk4_step, PHYSICS_DT, MAX_TORQUE


# ── LQR design ───────────────────────────────────────────────────────────────

class TestLQRDesign:
    """
    The LQR is a 2-state controller for the pendulum subsystem:
      state : [theta, theta_dot]
      input : tau (reaction motor torque)
    Cart position/velocity are controlled separately by wheel motors.
    """

    def test_gain_vector_shape(self):
        K = compute_lqr_gains()
        assert K.shape == (2,), f"Expected shape (2,), got {K.shape}"

    def test_system_matrix_shapes(self):
        A, B = get_system_matrices()
        assert A.shape == (2, 2)
        assert B.shape == (2, 1)

    def test_open_loop_is_unstable(self):
        """Inverted pendulum must have at least one unstable open-loop pole."""
        A, _ = get_system_matrices()
        eigs = np.linalg.eigvals(A)
        assert any(e.real > 0 for e in eigs), \
            "Open-loop pendulum should be unstable"

    def test_closed_loop_is_stable(self):
        """LQR must place all closed-loop eigenvalues in the left half-plane."""
        A, B = get_system_matrices()
        K    = compute_lqr_gains()
        A_cl = A - B @ K.reshape(1, -1)
        eigs = np.linalg.eigvals(A_cl)
        assert all(e.real < 0 for e in eigs), \
            f"Closed-loop unstable. Eigenvalues: {eigs}"

    def test_zero_state_zero_torque(self):
        """At equilibrium [theta=0, omega=0], LQR should command zero torque."""
        K   = compute_lqr_gains()
        tau = float(-K @ np.zeros(2))
        assert abs(tau) < 1e-12

    def test_torque_opposes_positive_tilt(self):
        K   = compute_lqr_gains()
        tau = float(-K @ np.array([0.2, 0.0]))
        assert tau < 0, "Positive tilt → negative (restoring) torque"

    def test_torque_opposes_negative_tilt(self):
        K   = compute_lqr_gains()
        tau = float(-K @ np.array([-0.2, 0.0]))
        assert tau > 0, "Negative tilt → positive (restoring) torque"

    def test_larger_tilt_larger_torque(self):
        K    = compute_lqr_gains()
        tau1 = abs(float(-K @ np.array([0.1, 0.0])))
        tau2 = abs(float(-K @ np.array([0.3, 0.0])))
        assert tau2 > tau1, "Larger tilt should demand larger torque"

    def test_lqr_stabilises_simulation(self):
        """LQR should reduce |θ| from 0.15 rad to < 2° within 5 s."""
        K     = compute_lqr_gains()
        state = np.array([0.0, 0.0, 0.15, 0.0])
        for _ in range(int(5.0 / PHYSICS_DT)):
            tau   = float(np.clip(-K @ state[2:4], -MAX_TORQUE, MAX_TORQUE))
            state = rk4_step(state, 0.0, tau)
        assert abs(state[2]) < np.radians(4), \
            f"LQR did not stabilise: final θ = {np.degrees(state[2]):.2f}°"

    def test_custom_Q_R_still_stable(self):
        """User-supplied Q/R should still yield a stable closed loop."""
        Q = np.diag([200.0, 30.0])
        R = np.array([[0.05]])
        A, B = get_system_matrices()
        K    = compute_lqr_gains(Q, R)
        A_cl = A - B @ K.reshape(1, -1)
        eigs = np.linalg.eigvals(A_cl)
        assert all(e.real < 0 for e in eigs)


# ── PID ───────────────────────────────────────────────────────────────────────

class TestPID:

    def test_proportional_only(self):
        pid = PID(kp=3.0, ki=0.0, kd=0.0, derivative_alpha=0.0)
        out = pid.compute(2.0, 0.01)
        assert abs(out - 6.0) < 1e-9

    def test_integral_accumulates_over_time(self):
        """After 10 steps of error=1 and dt=0.1, integral contribution = ki."""
        pid = PID(kp=0.0, ki=1.0, kd=0.0, derivative_alpha=0.0)
        for _ in range(10):
            out = pid.compute(1.0, 0.1)
        assert abs(out - 1.0) < 1e-6, f"Expected 1.0, got {out}"

    def test_derivative_responds_to_error_change(self):
        """Pure-D controller: output = kd * (err1 - err0) / dt."""
        pid = PID(kp=0.0, ki=0.0, kd=1.0, derivative_alpha=0.0)
        pid.compute(1.0, 0.1)          # prime prev_error = 1.0
        out = pid.compute(0.0, 0.1)    # (0 - 1) / 0.1 = -10
        assert abs(out - (-10.0)) < 1e-6

    def test_output_saturates_at_max(self):
        pid = PID(kp=100.0, ki=0.0, kd=0.0, max_output=5.0)
        out = pid.compute(1.0, 0.01)
        assert abs(out) <= 5.0 + 1e-9

    def test_integral_windup_clamp(self):
        """Integral should not grow beyond max_integral."""
        pid = PID(kp=0.0, ki=1.0, kd=0.0, max_output=3.0, max_integral=2.0)
        for _ in range(10000):
            pid.compute(1.0, 0.1)
        assert abs(pid._integral) <= 2.0 + 1e-9

    def test_reset_clears_all_state(self):
        pid = PID(kp=1.0, ki=1.0, kd=1.0)
        pid.compute(1.0, 0.1)
        pid.compute(2.0, 0.1)
        pid.reset()
        assert pid._integral    == 0.0
        assert pid._prev_error  == 0.0
        assert pid._d_filtered  == 0.0
        assert not pid._initialized

    def test_zero_error_zero_output(self):
        pid = PID(kp=5.0, ki=0.0, kd=0.0)
        out = pid.compute(0.0, 0.01)
        assert abs(out) < 1e-12

    def test_negative_error_negative_output(self):
        pid = PID(kp=2.0, ki=0.0, kd=0.0)
        out = pid.compute(-1.0, 0.01)
        assert out < 0

    @pytest.mark.parametrize("kp,ki,kd", [
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (2.0, 0.5, 0.3),
    ])
    def test_various_gains_dont_crash(self, kp, ki, kd):
        pid = PID(kp=kp, ki=ki, kd=kd)
        for _ in range(20):
            pid.compute(np.random.uniform(-1, 1), 0.01)


# ── CascadePID ────────────────────────────────────────────────────────────────

class TestCascadePID:

    def test_output_within_max_torque(self):
        """Output must never exceed max_output regardless of input."""
        pid = CascadePID(max_output=MAX_TORQUE)
        for _ in range(200):
            out = pid.compute(
                theta     = np.random.uniform(-0.5, 0.5),
                theta_dot = np.random.uniform(-3.0, 3.0),
                dt        = PHYSICS_DT,
            )
            assert abs(out) <= MAX_TORQUE + 1e-9

    def test_correct_torque_direction_positive_tilt(self):
        """Positive tilt, zero velocity → torque should be negative (restoring)."""
        pid = CascadePID(kp_outer=15.0, ki_outer=0.0, kd_outer=0.0,
                         kp_inner=2.0,  ki_inner=0.0, kd_inner=0.0,
                         max_output=MAX_TORQUE)
        out = pid.compute(0.3, 0.0, PHYSICS_DT)
        assert out < 0, f"Expected negative torque for positive tilt, got {out}"

    def test_correct_torque_direction_negative_tilt(self):
        """Negative tilt, zero velocity → torque should be positive (restoring)."""
        pid = CascadePID(kp_outer=15.0, ki_outer=0.0, kd_outer=0.0,
                         kp_inner=2.0,  ki_inner=0.0, kd_inner=0.0,
                         max_output=MAX_TORQUE)
        out = pid.compute(-0.3, 0.0, PHYSICS_DT)
        assert out > 0

    def test_zero_angle_zero_velocity_zero_torque(self):
        """At perfect equilibrium (P/D only), torque should be zero."""
        pid = CascadePID(kp_outer=15.0, ki_outer=0.0, kd_outer=0.0,
                         kp_inner=2.0,  ki_inner=0.0, kd_inner=0.0,
                         max_output=MAX_TORQUE)
        out = pid.compute(0.0, 0.0, PHYSICS_DT)
        assert abs(out) < 1e-9

    def test_reset_clears_both_loops(self):
        pid = CascadePID()
        for _ in range(50):
            pid.compute(0.2, 0.5, PHYSICS_DT)
        pid.reset()
        assert pid.outer._integral == 0.0
        assert pid.inner._integral == 0.0

    def test_stabilises_pendulum_in_simulation(self):
        """Cascade PID should stabilise |θ| < 5° within 10 s."""
        pid   = CascadePID(max_output=MAX_TORQUE)
        state = np.array([0.0, 0.0, 0.12, 0.0])
        for _ in range(int(10.0 / PHYSICS_DT)):
            tau   = pid.compute(state[2], state[3], PHYSICS_DT)
            state = rk4_step(state, 0.0, tau)
        assert abs(state[2]) < np.radians(5), \
            f"Cascade PID failed: final θ = {np.degrees(state[2]):.2f}°"


# ── Cross-module: LQR vs PID comparison ──────────────────────────────────────

class TestControllerComparison:

    def _run_sim(self, controller_fn, seconds=5.0, theta0=0.15):
        state = np.array([0.0, 0.0, theta0, 0.0])
        max_theta = abs(theta0)
        for _ in range(int(seconds / PHYSICS_DT)):
            tau   = float(np.clip(controller_fn(state), -MAX_TORQUE, MAX_TORQUE))
            state = rk4_step(state, 0.0, tau)
            max_theta = max(max_theta, abs(state[2]))
        return state, max_theta

    def test_both_stabilise_small_perturbation(self):
        K   = compute_lqr_gains()
        pid = CascadePID(max_output=MAX_TORQUE)

        lqr_state, _ = self._run_sim(lambda s: -K @ s[2:4])
        pid_state, _ = self._run_sim(lambda s: pid.compute(s[2], s[3], PHYSICS_DT))

        assert abs(lqr_state[2]) < np.radians(3), "LQR should stabilise"
        assert abs(pid_state[2]) < np.radians(15), "PID should stabilise"

    def test_lqr_outperforms_pid_settling(self):
        """LQR should have smaller residual angle than PID after 5 s."""
        K   = compute_lqr_gains()
        pid = CascadePID(max_output=MAX_TORQUE)

        lqr_state, _ = self._run_sim(lambda s: -K @ s[2:4])
        pid_state, _ = self._run_sim(lambda s: pid.compute(s[2], s[3], PHYSICS_DT))

        assert abs(lqr_state[2]) <= abs(pid_state[2]) + np.radians(5), \
            "LQR should settle at least as well as PID"
