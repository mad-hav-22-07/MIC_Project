"""
PID controllers for pendulum stabilisation.

PID        — single-loop PID with anti-windup and derivative filtering
CascadePID — two-loop cascade:
               outer: angle error  → angular-velocity setpoint
               inner: omega error  → motor torque
"""

import numpy as np


class PID:
    """
    Discrete PID with:
      • Clamped integral (anti-windup)
      • First-order derivative low-pass filter  (alpha = 0 → pure D, 1 → no D)
      • Symmetric output saturation
    """

    def __init__(self, kp: float, ki: float, kd: float,
                 max_output: float = None,
                 max_integral: float = None,
                 derivative_alpha: float = 0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output   = max_output
        self.max_integral = max_integral if max_integral is not None \
                            else (max_output if max_output is not None else 1e9)
        self.derivative_alpha = derivative_alpha   # smoothing for D term

        self._integral      = 0.0
        self._prev_error    = 0.0
        self._d_filtered    = 0.0
        self._initialized   = False

    # ── public ──────────────────────────────────────────────────────────────

    def reset(self):
        self._integral    = 0.0
        self._prev_error  = 0.0
        self._d_filtered  = 0.0
        self._initialized = False

    def compute(self, error: float, dt: float) -> float:
        if not self._initialized:
            self._prev_error  = error
            self._initialized = True

        # Integral with anti-windup clamp
        self._integral += error * dt
        self._integral  = float(np.clip(self._integral,
                                        -self.max_integral, self.max_integral))

        # Filtered derivative
        raw_d = (error - self._prev_error) / dt if dt > 1e-9 else 0.0
        a = self.derivative_alpha
        self._d_filtered = a * self._d_filtered + (1.0 - a) * raw_d
        self._prev_error = error

        output = (self.kp * error
                  + self.ki * self._integral
                  + self.kd * self._d_filtered)

        if self.max_output is not None:
            output = float(np.clip(output, -self.max_output, self.max_output))

        return output


class CascadePID:
    """
    Two-loop cascade PID for pendulum stabilisation.

    Outer loop  (angle → omega setpoint):
        error  = 0 - theta
        output = desired angular velocity

    Inner loop  (omega → torque):
        error  = omega_setpoint - theta_dot
        output = reaction motor torque (N·m)
    """

    def __init__(self,
                 kp_outer: float = 20.0,
                 ki_outer: float = 0.0,
                 kd_outer: float = 0.0,
                 kp_inner: float = 1.5,
                 ki_inner: float = 0.0,
                 kd_inner: float = 0.0,
                 max_output: float = 5.0):

        self.max_output = max_output
        max_omega = max_output * 2.0   # inner-loop headroom

        self.outer = PID(kp_outer, ki_outer, kd_outer,
                         max_output=max_omega,
                         max_integral=max_omega * 0.5)
        self.inner = PID(kp_inner, ki_inner, kd_inner,
                         max_output=max_output,
                         max_integral=max_output * 0.5)

    # ── public ──────────────────────────────────────────────────────────────

    def reset(self):
        self.outer.reset()
        self.inner.reset()

    def compute(self, theta: float, theta_dot: float, dt: float) -> float:
        """
        Parameters
        ----------
        theta     : pendulum angle (rad), 0 = upright
        theta_dot : pendulum angular velocity (rad/s)
        dt        : timestep (s)

        Returns
        -------
        torque : float — reaction motor torque command (N·m)
        """
        omega_setpoint = self.outer.compute(0.0 - theta, dt)
        torque         = self.inner.compute(omega_setpoint - theta_dot, dt)
        return torque
