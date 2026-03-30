"""
system_identification.py — Module 6
=====================================
Estimates the real physical parameters of the pendulum from hardware data,
closing the simulation-to-reality gap.

Identified parameters
---------------------
  I_hinge  — moment of inertia of the pendulum rod about the hinge (kg·m²)
  m_eff    — effective pendulum mass (kg)  (accounts for mass distribution)
  b_pend   — rotational friction at hinge (N·m·s/rad)

Procedure
---------
1. Pendulum is held near vertical, then a brief step torque is applied by
   the reaction motor while θ and ω are recorded from the BNO055.
2. The 2-state linear pendulum ODE is integrated numerically (scipy RK45).
3. scipy.optimize.least_squares minimises the RMS error between the
   simulated and measured θ(t), fitting the three parameters above.
4. Updated A, B matrices are printed and new LQR gains are computed.

Modes
-----
  Offline (default) — generates synthetic noisy data and fits to it.
                      Use this to verify the fitting pipeline before hardware.
  Live (--live)     — collects a real step-response from the ESP32 via WiFi.
                      The ESP32 must be running main.cpp with WiFi connected.

Usage
-----
  python sysid/system_identification.py              # offline verification
  python sysid/system_identification.py --live       # real hardware
  python sysid/system_identification.py --live --esp32 192.168.1.42
"""

import sys
import os
import argparse
import socket
import json
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are

# Add simulation/ to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "simulation"))
from lqr_design import compute_lqr_gains

# ── Network defaults ─────────────────────────────────────────────────────────
ESP32_IP   = "192.168.1.100"
CMD_PORT   = 4210
TEL_PORT   = 4211

# ── Step test parameters ──────────────────────────────────────────────────────
STEP_TORQUE_NM  = 0.8     # N·m — small enough to stay in the linear regime
STEP_DURATION_S = 0.4     # seconds the step torque is held on
RECORD_SECS     = 3.0     # total data-collection window
GRAVITY         = 9.81

# ── Nominal simulation parameters (from pendulum_sim.py / lqr_design.py) ─────
NOMINAL = {
    "I_hinge": 0.025,    # 1/3 * 0.3 * 0.5**2
    "m_eff"  : 0.3,
    "b_pend" : 0.01,
    "L"      : 0.5,
    "g"      : GRAVITY,
}
NOMINAL_ALPHA = NOMINAL["m_eff"] * GRAVITY * NOMINAL["L"] / 2.0


# ─── Linear pendulum ODE ─────────────────────────────────────────────────────

def pendulum_ode(t_scalar, y, I_h, alpha, b, times_arr, tau_arr):
    """
    Linearised single-DOF pendulum ODE for parameter fitting.

    State: y = [theta, theta_dot]
    EOM:   I_h * theta_ddot = alpha * theta - b * theta_dot + tau(t)

    where alpha = m_eff * g * L/2  (gravity torque coefficient, positive
    because upright is unstable — positive tilt → positive theta_ddot).

    Parameters
    ----------
    I_h     : hinge moment of inertia  (kg·m²)
    alpha   : gravity torque coefficient = m*g*L/2  (N·m/rad)
    b       : viscous friction (N·m·s/rad)
    times_arr, tau_arr : torque lookup table (zero-order hold)
    """
    th, thd = y
    idx  = int(np.searchsorted(times_arr, t_scalar))
    idx  = min(idx, len(tau_arr) - 1)
    tau  = float(tau_arr[idx])
    thdd = (alpha * th - b * thd + tau) / I_h
    return [thd, thdd]


# ─── Parameter fitter ────────────────────────────────────────────────────────

class ParameterFitter:
    """
    Fits {I_hinge, alpha, b_pend} to measured θ(t) via least-squares.

    The fit is parameterised as [I_h, alpha, b] where alpha = m_eff*g*L/2.
    After fitting, m_eff is recovered using the nominal rod length.
    """

    def simulate(self, params: np.ndarray, times: np.ndarray,
                 tau_arr: np.ndarray) -> np.ndarray:
        """
        Integrate the pendulum ODE with the given parameter vector.
        Returns simulated theta(t) array (same length as times).
        """
        I_h, alpha, b = params
        sol = solve_ivp(
            pendulum_ode,
            t_span=[times[0], times[-1]],
            y0=[0.0, 0.0],
            t_eval=times,
            args=(I_h, alpha, b, times, tau_arr),
            method="RK45",
            rtol=1e-4, atol=1e-6,
            max_step=0.005,
        )
        return sol.y[0]    # theta trajectory

    def fit(self,
            times: np.ndarray,
            theta_meas: np.ndarray,
            tau_arr: np.ndarray,
            p0: tuple = None) -> dict:
        """
        Fit model parameters to measured data.

        Parameters
        ----------
        times      : sample time array (s), monotonically increasing
        theta_meas : measured pendulum angle (rad)
        tau_arr    : torque command at each sample time (N·m)
        p0         : initial guess [I_hinge, alpha, b_pend]
                     (defaults to nominal simulation values)

        Returns
        -------
        dict with keys:
            I_hinge, m_eff, b_pend, alpha — fitted values
            residual_rms                  — RMS fit error (rad)
            success                       — bool, solver converged
        """
        if p0 is None:
            p0 = (NOMINAL["I_hinge"], NOMINAL_ALPHA, NOMINAL["b_pend"])

        def residuals(params):
            # Return Inf if any parameter violates physical constraints
            if any(p <= 0 for p in params):
                return np.ones_like(theta_meas) * 1e6
            theta_sim = self.simulate(params, times, tau_arr)
            return theta_sim - theta_meas

        result = least_squares(
            residuals, p0,
            bounds=([1e-4, 0.01, 0.0], [2.0, 20.0, 2.0]),
            method="trf",
            verbose=0,
        )

        I_h, alpha_fit, b = result.x
        # Recover m_eff from alpha = m_eff * g * L/2 using nominal rod length
        m_eff = (2.0 * alpha_fit) / (GRAVITY * NOMINAL["L"])

        return {
            "I_hinge"      : float(I_h),
            "m_eff"        : float(m_eff),
            "b_pend"       : float(b),
            "alpha"        : float(alpha_fit),
            "residual_rms" : float(np.sqrt(np.mean(result.fun**2))),
            "success"      : bool(result.success),
        }


# ─── Data collection (offline simulation) ────────────────────────────────────

def generate_synthetic_data(noise_std_rad: float = 0.002) -> tuple:
    """
    Generate noisy synthetic step-response data using the nominal parameters.
    Used to verify the fitting pipeline without hardware.

    Returns (times, theta_rad, tau_arr) arrays.
    """
    dt   = 1.0 / 500.0   # 500 Hz
    t    = np.arange(0.0, RECORD_SECS, dt)
    tau  = np.where(t < STEP_DURATION_S, STEP_TORQUE_NM, 0.0)

    def ode(t_, y):
        th, thd = y
        idx  = min(int(t_ / dt), len(tau) - 1)
        thdd = (NOMINAL_ALPHA * th - NOMINAL["b_pend"] * thd + tau[idx]) / NOMINAL["I_hinge"]
        return [thd, thdd]

    sol   = solve_ivp(ode, [t[0], t[-1]], [0.0, 0.0], t_eval=t,
                      method="RK45", rtol=1e-5)
    theta = sol.y[0] + np.random.normal(0.0, noise_std_rad, len(t))

    return t, theta, tau


# ─── Data collection (live hardware) ─────────────────────────────────────────

def collect_live_data(esp32_ip: str) -> tuple:
    """
    Command a step torque on the ESP32 and record IMU telemetry.

    The ESP32 must support the "TORQUE <N·m>" command (implemented in
    wifi_comm.cpp).  Only use this with the pendulum near vertical and free
    to rotate.

    Returns (times, theta_rad, tau_arr) arrays.
    """
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tel_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tel_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tel_sock.bind(("0.0.0.0", TEL_PORT))
    tel_sock.settimeout(0.03)

    times_list    = []
    theta_list    = []
    tau_list      = []

    t_start = time.time()
    print(f"Collecting live data from {esp32_ip} for {RECORD_SECS:.0f} s...")

    while time.time() - t_start < RECORD_SECS:
        elapsed = time.time() - t_start

        # Step torque on for STEP_DURATION_S, then off
        if elapsed < STEP_DURATION_S:
            tau = STEP_TORQUE_NM
            cmd_sock.sendto(f"TORQUE {tau:.3f}".encode(), (esp32_ip, CMD_PORT))
        else:
            tau = 0.0
            cmd_sock.sendto(b"STOP", (esp32_ip, CMD_PORT))

        try:
            data, _ = tel_sock.recvfrom(512)
            pkt      = json.loads(data.decode())
            t_s      = pkt.get("t", 0) / 1000.0
            th_deg   = pkt.get("th", 0.0)
            times_list.append(t_s)
            theta_list.append(np.radians(th_deg))
            tau_list.append(tau)
        except (socket.timeout, json.JSONDecodeError):
            pass

    cmd_sock.sendto(b"STOP", (esp32_ip, CMD_PORT))
    cmd_sock.close()
    tel_sock.close()

    # Sort by time (UDP packets can arrive slightly out of order)
    if not times_list:
        raise RuntimeError("No telemetry received.  Check ESP32 IP and WiFi.")

    idx   = np.argsort(times_list)
    t_arr = np.array(times_list)[idx] - times_list[0]   # zero-reference
    th_arr = np.array(theta_list)[idx]
    tau_arr = np.array(tau_list)[idx]

    return t_arr, th_arr, tau_arr


# ─── Results printer ─────────────────────────────────────────────────────────

def print_results(result: dict, K_nominal: np.ndarray, K_fitted: np.ndarray):
    print("\n" + "=" * 58)
    print("  System Identification Results")
    print("=" * 58)
    print(f"  Convergence  : {'YES' if result['success'] else 'NO (treat results with caution)'}")
    print(f"  RMS residual : {result['residual_rms']*1000:.2f} mrad "
          f"({np.degrees(result['residual_rms']):.3f} °)")
    print()
    print(f"  {'Parameter':<14}  {'Fitted':>10}  {'Nominal':>10}  {'Δ (%)':>8}")
    print("  " + "-" * 48)
    for name, fit_val, nom_val in [
        ("I_hinge (kg·m²)", result["I_hinge"], NOMINAL["I_hinge"]),
        ("m_eff (kg)",       result["m_eff"],   NOMINAL["m_eff"]),
        ("b_pend",           result["b_pend"],  NOMINAL["b_pend"]),
    ]:
        delta = 100.0 * (fit_val - nom_val) / nom_val if nom_val != 0 else 0.0
        print(f"  {name:<14}  {fit_val:>10.4f}  {nom_val:>10.4f}  {delta:>+7.1f}%")
    print()
    print(f"  LQR K (nominal) : [{K_nominal[0]:.3f},  {K_nominal[1]:.3f}]")
    print(f"  LQR K (fitted)  : [{K_fitted[0]:.3f},  {K_fitted[1]:.3f}]")
    print()
    print(f"  → Send to ESP32:  LQR {K_fitted[0]:.4f} {K_fitted[1]:.4f}")
    print("=" * 58)


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Inverted Pendulum System Identification")
    parser.add_argument("--live", action="store_true",
                        help="Collect data from real ESP32 hardware via WiFi")
    parser.add_argument("--esp32", default=ESP32_IP,
                        help=f"ESP32 IP address (default: {ESP32_IP})")
    args = parser.parse_args()

    # ── Data collection ──────────────────────────────────────────────────────
    if args.live:
        print("=== LIVE MODE ===")
        print("WARNING: pendulum must be near vertical and free to rotate.")
        print(f"         Applying {STEP_TORQUE_NM} N·m step for {STEP_DURATION_S} s.")
        input("Press ENTER to begin (Ctrl-C to abort)...")
        times, theta_rad, tau_arr = collect_live_data(args.esp32)
    else:
        print("=== OFFLINE MODE — synthetic data with Gaussian noise ===")
        times, theta_rad, tau_arr = generate_synthetic_data()

    print(f"Collected {len(times)} samples,  duration: {times[-1]-times[0]:.2f} s")

    # ── Fitting ──────────────────────────────────────────────────────────────
    print("Fitting parameters via least squares...")
    fitter = ParameterFitter()
    result = fitter.fit(times, theta_rad, tau_arr)

    # ── Recompute LQR with fitted plant model ────────────────────────────────
    I_fit   = result["I_hinge"]
    m_fit   = result["m_eff"]
    b_fit   = result["b_pend"]
    L_nom   = NOMINAL["L"]

    A_fit = np.array([
        [0.0,                                 1.0          ],
        [m_fit * GRAVITY * L_nom / (2*I_fit), -b_fit/I_fit ],
    ])
    B_fit = np.array([[0.0], [1.0 / I_fit]])

    Q = np.diag([120.0, 20.0])
    R = np.array([[0.01]])
    P = solve_continuous_are(A_fit, B_fit, Q, R)
    K_fitted  = (np.linalg.inv(R) @ B_fit.T @ P).flatten()
    K_nominal = compute_lqr_gains()

    print_results(result, K_nominal, K_fitted)

    # ── Optional plot ────────────────────────────────────────────────────────
    try:
        import matplotlib.pyplot as plt

        theta_sim = fitter.simulate(
            np.array([result["I_hinge"], result["alpha"], result["b_pend"]]),
            times, tau_arr,
        )

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 5), tight_layout=True)
        fig.patch.set_facecolor("#12151c")
        for ax in (ax1, ax2):
            ax.set_facecolor("#161a22")
            ax.tick_params(colors="#aaaaaa")

        ax1.plot(times, np.degrees(theta_rad), "c-",  lw=1.5, label="Measured")
        ax1.plot(times, np.degrees(theta_sim), "r--", lw=1.5, label="Fitted model")
        ax1.set_ylabel("θ (°)", color="#aaaaaa")
        ax1.set_title("Step Response Fit", color="#dddddd")
        ax1.legend(facecolor="#2a2e3a", labelcolor="#dddddd")

        residual = np.degrees(theta_rad - theta_sim)
        ax2.plot(times, residual, "orange", lw=1.0)
        ax2.axhline(0, color="white", lw=0.5, ls="--")
        ax2.set_ylabel("Residual (°)", color="#aaaaaa")
        ax2.set_xlabel("Time (s)", color="#aaaaaa")
        ax2.set_title("Residual", color="#dddddd")

        rms_deg = np.degrees(result["residual_rms"])
        plt.suptitle(f"System ID — RMS residual: {rms_deg:.3f}°",
                     color="#dddddd", fontsize=10)
        plt.show()
    except ImportError:
        print("(matplotlib not available — skipping plot)")


if __name__ == "__main__":
    main()
