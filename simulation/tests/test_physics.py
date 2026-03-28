"""
Tests for physics engine in pendulum_sim.py

Run from the project root:
    pytest simulation/tests/test_physics.py -v
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import pytest
import pendulum_sim as sim
from pendulum_sim import (
    equations_of_motion, rk4_step, simulate_imu,
    PHYSICS_DT, MAX_TORQUE,
    CART_MASS, PEND_MASS, PEND_LEN, GRAVITY, I_HINGE,
)


# ── Equations of motion ───────────────────────────────────────────────────────

class TestEquationsOfMotion:

    def test_upright_no_force_zero_acceleration(self):
        """At upright equilibrium with no inputs, accelerations must be zero."""
        state = np.array([0.0, 0.0, 0.0, 0.0])
        sd    = equations_of_motion(state, 0.0, 0.0)
        assert abs(sd[1]) < 1e-10, f"x_ddot should be 0, got {sd[1]}"
        assert abs(sd[3]) < 1e-10, f"θ_ddot should be 0, got {sd[3]}"

    def test_velocity_passthrough(self):
        """sd[0] == x_dot and sd[2] == theta_dot by definition."""
        state = np.array([1.5, 2.3, 0.1, -0.7])
        sd    = equations_of_motion(state, 0.0, 0.0)
        assert abs(sd[0] - 2.3) < 1e-12
        assert abs(sd[2] - (-0.7)) < 1e-12

    def test_gravity_makes_pendulum_fall(self):
        """Small positive tilt: gravity should produce positive θ_ddot (unstable)."""
        state = np.array([0.0, 0.0, 0.1, 0.0])
        sd    = equations_of_motion(state, 0.0, 0.0)
        assert sd[3] > 0, "Gravity should accelerate an upward tilt further"

    def test_stabilising_torque_reduces_theta_ddot(self):
        """Opposing torque should reduce |θ_ddot| compared to free fall."""
        state = np.array([0.0, 0.0, 0.1, 0.0])
        sd0   = equations_of_motion(state,  0.0,  0.0)
        sd_t  = equations_of_motion(state,  0.0, -3.0)   # negative = restoring
        assert sd_t[3] < sd0[3], "Restoring torque should reduce θ_ddot"

    def test_cart_force_accelerates_cart(self):
        """Positive F → positive x_ddot; negative F → negative x_ddot."""
        state = np.array([0.0, 0.0, 0.0, 0.0])
        sd_pos = equations_of_motion(state,  15.0, 0.0)
        sd_neg = equations_of_motion(state, -15.0, 0.0)
        assert sd_pos[1] > 0
        assert sd_neg[1] < 0

    def test_cart_friction_opposes_velocity(self):
        """Cart friction should decelerate a moving cart."""
        state_r = np.array([0.0,  1.0, 0.0, 0.0])
        state_l = np.array([0.0, -1.0, 0.0, 0.0])
        sd_r = equations_of_motion(state_r, 0.0, 0.0)
        sd_l = equations_of_motion(state_l, 0.0, 0.0)
        assert sd_r[1] < 0, "Friction should decelerate rightward cart"
        assert sd_l[1] > 0, "Friction should decelerate leftward cart"

    def test_returns_four_element_array(self):
        state = np.array([0.0, 0.0, 0.05, 0.2])
        sd    = equations_of_motion(state, 5.0, 1.0)
        assert sd.shape == (4,)

    def test_antisymmetry_in_theta(self):
        """EOM should be antisymmetric: flipping (theta, torque) flips accelerations."""
        state_p = np.array([0.0, 0.0,  0.2,  0.0])
        state_n = np.array([0.0, 0.0, -0.2,  0.0])
        sd_p    = equations_of_motion(state_p, 0.0,  2.0)
        sd_n    = equations_of_motion(state_n, 0.0, -2.0)
        assert abs(sd_p[3] + sd_n[3]) < 1e-10, \
            "θ_ddot should be antisymmetric in θ and τ"


# ── RK4 integrator ────────────────────────────────────────────────────────────

class TestRK4:

    def test_returns_correct_shape(self):
        state = np.array([0.0, 0.0, 0.05, 0.0])
        ns    = rk4_step(state, 0.0, 0.0)
        assert ns.shape == (4,)

    def test_uncontrolled_pendulum_falls(self):
        """Tilted pendulum with no control should diverge over 0.5 s."""
        state = np.array([0.0, 0.0, 0.1, 0.0])
        for _ in range(int(0.5 / PHYSICS_DT)):
            state = rk4_step(state, 0.0, 0.0)
        assert abs(state[2]) > 0.15, "Uncontrolled pendulum should fall further"

    def test_energy_approximately_conserved_no_damping(self, monkeypatch):
        """With damping zeroed, mechanical energy should drift < 1 % over 1 s."""
        monkeypatch.setattr(sim, "CART_FRIC", 0.0)
        monkeypatch.setattr(sim, "PEND_FRIC", 0.0)
        monkeypatch.setattr(sim, "PEND_AIR",  0.0)

        state = np.array([0.0, 0.0, 0.2, 0.0])

        def energy(s):
            # Lagrangian-consistent total energy for cart + uniform rod.
            # KE = 0.5*(M+m)*xd² + m*xd*(L/2)*thd*cos(th) + 0.5*I_hinge*thd²
            # PE = m*g*(L/2)*cos(th)
            _, xd, th, thd = s
            M, m, L, g = CART_MASS, PEND_MASS, PEND_LEN, GRAVITY
            KE = (0.5 * (M + m) * xd**2
                  + m * xd * (L / 2) * thd * np.cos(th)
                  + 0.5 * I_HINGE * thd**2)
            PE = m * g * (L / 2) * np.cos(th)
            return KE + PE

        E0 = energy(state)
        for _ in range(int(1.0 / PHYSICS_DT)):
            state = rk4_step(state, 0.0, 0.0)
        E1 = energy(state)

        drift = abs(E1 - E0) / (abs(E0) + 1e-9)
        assert drift < 0.01, f"Energy drift too large: {drift*100:.3f} %"

    def test_lqr_stabilises_from_small_perturbation(self):
        """LQR closed loop should settle to |θ| < 2° within 5 s."""
        from lqr_design import compute_lqr_gains
        K     = compute_lqr_gains()
        state = np.array([0.0, 0.0, 0.15, 0.0])
        for _ in range(int(5.0 / PHYSICS_DT)):
            tau   = float(np.clip(-K @ state[2:4], -MAX_TORQUE, MAX_TORQUE))
            state = rk4_step(state, 0.0, tau)
        assert abs(state[2]) < np.radians(4), \
            f"LQR failed to stabilise: θ = {np.degrees(state[2]):.2f}°"

    def test_small_dt_limit(self):
        """rk4_step with very small dt should return state close to input."""
        state = np.array([0.5, 1.0, 0.05, 0.2])
        ns    = rk4_step(state, 5.0, 1.0, dt=1e-6)
        assert np.allclose(ns, state, atol=1e-4)


# ── Simulated IMU ─────────────────────────────────────────────────────────────

class TestSimulatedIMU:

    def test_angle_field_matches_state(self):
        state = np.array([0.0, 0.5, 0.35, 0.1])
        imu   = simulate_imu(state, state)
        assert abs(imu["euler_z"] - np.degrees(0.35)) < 1e-6

    def test_gyro_field_matches_state(self):
        state = np.array([0.0, 0.0, 0.0, 2.1])
        imu   = simulate_imu(state, state)
        assert abs(imu["gyro_z"] - np.degrees(2.1)) < 1e-6

    def test_accel_x_from_cart_velocity_change(self):
        """accel_x should reflect cart velocity change over dt."""
        state_prev = np.array([0.0, 0.0, 0.0, 0.0])
        state      = np.array([0.0, 1.0, 0.0, 0.0])   # xd went 0→1 in PHYSICS_DT
        imu = simulate_imu(state, state_prev)
        expected = 1.0 / PHYSICS_DT
        assert abs(imu["accel_x"] - expected) < 1e-3

    def test_static_accel_z_near_gravity(self):
        """At rest (no angular acceleration), accel_z ≈ g."""
        state = np.array([0.0, 0.0, 0.0, 0.0])
        imu   = simulate_imu(state, state)
        assert abs(imu["accel_z"] - GRAVITY) < 0.5

    def test_all_required_keys_present(self):
        state = np.array([0.0, 0.0, 0.0, 0.0])
        imu   = simulate_imu(state, state)
        for key in ("euler_z", "gyro_z", "accel_x", "accel_z"):
            assert key in imu, f"Missing IMU key: {key}"

    def test_upright_zero_angle(self):
        state = np.array([1.5, -0.3, 0.0, 0.0])
        imu   = simulate_imu(state, state)
        assert abs(imu["euler_z"]) < 1e-9

    def test_negative_tilt_gives_negative_angle(self):
        state = np.array([0.0, 0.0, -0.25, 0.0])
        imu   = simulate_imu(state, state)
        assert imu["euler_z"] < 0
