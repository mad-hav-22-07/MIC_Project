"""
race_controller.py — Race Mode Controller (Module 7)
======================================================
Maximises cart speed along the track while the LQR stabiliser keeps
the pendulum upright.

Strategy
--------
1. Velocity profile  — trapezoidal ramp: accelerate at max rate to a cruise
   speed, then decelerate in time to stop at the track end.  Position is
   estimated by dead-reckoning (no encoder feedback assumed).

2. Feedforward torque — the planned cart acceleration creates an inertial
   pseudo-force that tilts the pendulum.  A feedforward torque offset
   pre-compensates this disturbance before it builds up, so the LQR sees
   a smaller error to correct.

     τ_ff = −(m · L/2) · ẍ_cart
     (from the cart-pendulum coupling term in the EOM)

3. Stability guard — if |θ| exceeds STAB_MARGIN_DEG, the cart slows to a
   fraction of cruise speed and waits for the pendulum to recover below
   RECOVER_THRESH_DEG before resuming.

Modes
-----
  Simulation (default) — runs a full race in the physics simulator and
                          plots position, speed, angle, and cart force.
  Live (--live)        — sends real-time DRIVE commands to the ESP32 over
                          WiFi using dead-reckoning position estimates.

Usage
-----
  python race_mode/race_controller.py              # simulation demo + plot
  python race_mode/race_controller.py --live
  python race_mode/race_controller.py --live --esp32 192.168.1.42
  python race_mode/race_controller.py --vmax 2.0   # tune cruise speed
"""

import sys
import os
import argparse
import socket
import json
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "simulation"))
from pendulum_sim import (
    rk4_step, PHYSICS_DT, MAX_TORQUE, MAX_CART_FORCE,
    TRACK_HALF, CART_MASS, PEND_MASS, PEND_LEN,
)
from lqr_design import compute_lqr_gains

# ── Race defaults ─────────────────────────────────────────────────────────────
VMAX_DEFAULT      = 3.0    # m/s cruise speed
ACCEL_DEFAULT     = 1.5    # m/s² acceleration ramp rate
DECEL_DEFAULT     = 2.0    # m/s² braking ramp rate
STAB_MARGIN_DEG   = 10.0   # degree threshold → trigger slowdown
RECOVER_THRESH_DEG = 5.0   # degree threshold → resume full speed
RECOVER_SPEED_FRAC = 0.3   # fraction of vmax during stability-guard recovery

ESP32_IP  = "192.168.1.100"
CMD_PORT  = 4210
TEL_PORT  = 4211


# ─── Trapezoidal velocity profile ────────────────────────────────────────────

class VelocityProfile:
    """
    Compute desired speed and acceleration at any track position.

    The profile has three phases:
      Ramp up   : speed increases linearly from 0 at constant acceleration.
      Cruise    : speed held at v_max.
      Brake     : speed decreases linearly to 0 at x_end.

    v_max is reduced automatically if the track is too short to reach it
    before braking must begin (triangular profile).
    """

    def __init__(self, x_start: float, x_end: float,
                 v_max: float, accel: float, decel: float):
        self.x_start = x_start
        self.x_end   = x_end
        self.v_max   = v_max
        self.accel   = accel
        self.decel   = decel
        self.length  = x_end - x_start

        # Minimum track length needed for full trapezoidal profile
        d_up   = v_max**2 / (2 * accel)
        d_down = v_max**2 / (2 * decel)
        if d_up + d_down > self.length:
            # Track too short — reduce to triangular profile
            # v_peak from: v²/2a + v²/2d = L  →  v = sqrt(2*L/(1/a + 1/d))
            self.v_max = float(np.sqrt(
                2 * self.length / (1.0 / accel + 1.0 / decel)
            ))

    def target_speed(self, x: float) -> float:
        """Desired speed (m/s) at position x.  Clamped to [0, v_max]."""
        dist_from_start = x - self.x_start
        dist_to_end     = self.x_end - x

        v_up   = float(np.sqrt(max(0.0, 2 * self.accel * dist_from_start)))
        v_down = float(np.sqrt(max(0.0, 2 * self.decel * dist_to_end)))
        return float(np.clip(min(v_up, v_down, self.v_max), 0.0, self.v_max))

    def target_accel(self, x: float, v: float) -> float:
        """Desired acceleration (m/s²) given position x and current speed v."""
        v_tgt = self.target_speed(x)
        # Proportional chase of the speed setpoint — smoother than bang-bang
        a = 3.0 * (v_tgt - v)
        return float(np.clip(a, -self.decel, self.accel))


# ─── Feedforward torque predictor ────────────────────────────────────────────

class FeedforwardPredictor:
    """
    Compute the torque offset that pre-compensates the inertial pendulum
    disturbance caused by planned cart acceleration.

    From the full nonlinear EOM coupling term (linearised for small θ):
        τ_ff = −(m · L/2) · ẍ_cart

    Adding τ_ff to the LQR output means the reaction motor acts before the
    pendulum actually starts to tilt, reducing peak angle deviation during
    acceleration and braking.
    """

    def __init__(self, pend_mass: float = PEND_MASS, pend_len: float = PEND_LEN):
        self._coeff = pend_mass * pend_len / 2.0

    def compute(self, cart_accel: float) -> float:
        """
        Parameters
        ----------
        cart_accel : planned cart acceleration (m/s²), positive = forward

        Returns
        -------
        tau_ff : feedforward torque (N·m)
        """
        return float(-self._coeff * cart_accel)


# ─── Race controller ─────────────────────────────────────────────────────────

class RaceController:
    """
    Full race controller: velocity profile + feedforward + stability guard.

    Usage
    -----
        race = RaceController(x_start=-4.0, x_end=4.0)
        while not race.is_finished(x):
            F_cart, tau_ff = race.step(state, dt)
    """

    def __init__(self,
                 x_start: float       = -TRACK_HALF,
                 x_end:   float       =  TRACK_HALF,
                 v_max:   float       = VMAX_DEFAULT,
                 accel:   float       = ACCEL_DEFAULT,
                 decel:   float       = DECEL_DEFAULT,
                 stab_margin_deg:     float = STAB_MARGIN_DEG,
                 recover_thresh_deg:  float = RECOVER_THRESH_DEG,
                 recover_speed_frac:  float = RECOVER_SPEED_FRAC):

        self._profile          = VelocityProfile(x_start, x_end, v_max, accel, decel)
        self._ff               = FeedforwardPredictor()
        self._stab_margin      = np.radians(stab_margin_deg)
        self._recover_thresh   = np.radians(recover_thresh_deg)
        self._recover_frac     = recover_speed_frac
        self._recovering       = False

    def step(self, state: np.ndarray, dt: float) -> tuple:
        """
        Compute cart force and feedforward torque for the current state.

        Parameters
        ----------
        state : numpy array [x, x_dot, theta, theta_dot]
        dt    : timestep (s)

        Returns
        -------
        (F_cart, tau_ff) : cart force (N) and feedforward torque (N·m)
        """
        x, xd, th, _ = state

        # ── Stability guard ──────────────────────────────────────────────────
        if abs(th) > self._stab_margin:
            self._recovering = True
        elif abs(th) < self._recover_thresh:
            self._recovering = False

        # ── Target acceleration ──────────────────────────────────────────────
        if self._recovering:
            # Slow to recover_frac of cruise speed; simple proportional catch-up
            v_target = self._profile.v_max * self._recover_frac
            a_target = float(np.clip(3.0 * (v_target - xd),
                                     -DECEL_DEFAULT, ACCEL_DEFAULT))
        else:
            a_target = self._profile.target_accel(x, xd)

        # ── Cart force ───────────────────────────────────────────────────────
        # F = M·a; friction is corrected by the closed-loop controller on the ESP32.
        F_cart = float(np.clip(CART_MASS * a_target, -MAX_CART_FORCE, MAX_CART_FORCE))

        # ── Feedforward torque ───────────────────────────────────────────────
        tau_ff = self._ff.compute(a_target)

        return F_cart, tau_ff

    def is_finished(self, x: float) -> bool:
        """True once the cart reaches within 5 cm of the track end."""
        return x >= self._profile.x_end - 0.05

    def is_recovering(self) -> bool:
        return self._recovering


# ─── Simulation demo ─────────────────────────────────────────────────────────

def run_simulation(v_max: float):
    """Run a complete race in the simulator and plot results."""
    K    = compute_lqr_gains()
    race = RaceController(v_max=v_max)

    # Start at left end of track, pendulum perfectly upright
    state = np.array([-TRACK_HALF + 0.05, 0.0, 0.0, 0.0])

    log = dict(t=[], x=[], v=[], theta_deg=[], F_cart=[], tau_ff=[])
    t   = 0.0
    dt  = PHYSICS_DT

    print(f"Simulating race... (v_max={v_max} m/s)")

    while not race.is_finished(state[0]) and t < 60.0:
        F_cart, tau_ff = race.step(state, dt)
        tau_lqr   = float(np.clip(-K @ state[2:4], -MAX_TORQUE, MAX_TORQUE))
        tau_total = float(np.clip(tau_lqr + tau_ff, -MAX_TORQUE, MAX_TORQUE))

        state = rk4_step(state, F_cart, tau_total, dt)
        t    += dt

        log["t"].append(t)
        log["x"].append(state[0])
        log["v"].append(state[1])
        log["theta_deg"].append(np.degrees(state[2]))
        log["F_cart"].append(F_cart)
        log["tau_ff"].append(tau_ff)

    finish_time  = t
    max_tilt     = max(abs(np.array(log["theta_deg"])))
    max_speed    = max(log["v"])

    print(f"Finish time : {finish_time:.3f} s")
    print(f"Max speed   : {max_speed:.2f} m/s")
    print(f"Max tilt    : {max_tilt:.2f}°")

    try:
        import matplotlib.pyplot as plt

        t_arr     = np.array(log["t"])
        plots = [
            (np.array(log["x"]),          "Position (m)",    "cyan"),
            (np.array(log["v"]),          "Speed (m/s)",     "lime"),
            (np.array(log["theta_deg"]), "Tilt θ (°)",       "orange"),
            (np.array(log["F_cart"]),    "Cart force (N)",   "violet"),
            (np.array(log["tau_ff"]),    "Feedforward τ (N·m)", "yellow"),
        ]

        fig, axes = plt.subplots(len(plots), 1, figsize=(11, 9),
                                 sharex=True, tight_layout=True)
        fig.patch.set_facecolor("#12151c")

        for ax, (data, ylabel, col) in zip(axes, plots):
            ax.set_facecolor("#161a22")
            ax.plot(t_arr, data, col, lw=1.2)
            ax.set_ylabel(ylabel, color="#aaaaaa", fontsize=8)
            ax.axhline(0, color="white", lw=0.4, ls="--", alpha=0.4)
            ax.tick_params(colors="#888899", labelsize=7)

        # Mark stability margin on tilt plot
        axes[2].axhline( STAB_MARGIN_DEG, color="red", lw=0.7, ls=":", alpha=0.7)
        axes[2].axhline(-STAB_MARGIN_DEG, color="red", lw=0.7, ls=":", alpha=0.7)

        axes[-1].set_xlabel("Time (s)", color="#aaaaaa", fontsize=8)
        plt.suptitle(
            f"Race Mode — v_max={v_max} m/s  |  Finish: {finish_time:.2f} s  "
            f"|  Max tilt: {max_tilt:.1f}°",
            color="#dddddd", fontsize=10,
        )
        plt.show()
    except ImportError:
        print("(matplotlib not available — skipping plot)")


# ─── Live mode ───────────────────────────────────────────────────────────────

def run_live(esp32_ip: str, v_max: float):
    """
    Send real-time DRIVE commands to the ESP32 based on the race controller.
    Position is estimated by dead-reckoning (no encoder feedback).

    Note: dead-reckoning drifts over time; for a real competition, add wheel
    encoders and report position in the telemetry packet.
    """
    K    = compute_lqr_gains()
    race = RaceController(v_max=v_max)

    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tel_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tel_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tel_sock.bind(("0.0.0.0", TEL_PORT))
    tel_sock.settimeout(0.03)

    # Push LQR gains to make sure the ESP32 has the right values
    cmd_sock.sendto(f"LQR {K[0]:.4f} {K[1]:.4f}".encode(), (esp32_ip, CMD_PORT))
    cmd_sock.sendto(b"MODE LQR", (esp32_ip, CMD_PORT))
    print(f"Race mode LIVE — {esp32_ip}  (v_max={v_max} m/s).  Ctrl-C to stop.")

    # Dead-reckoning state
    x_est  = -TRACK_HALF + 0.05
    xd_est = 0.0
    th_rad = 0.0
    thd_rads = 0.0

    t_prev = time.time()

    try:
        while not race.is_finished(x_est):
            # Receive latest telemetry for θ, ω
            try:
                data, _ = tel_sock.recvfrom(512)
                pkt      = json.loads(data.decode())
                th_rad   = np.radians(pkt.get("th", 0.0))
                thd_rads = np.radians(pkt.get("om", 0.0))
            except (socket.timeout, json.JSONDecodeError):
                pass

            t_now = time.time()
            dt    = min(t_now - t_prev, 0.1)   # cap dt for robustness
            t_prev = t_now

            state = np.array([x_est, xd_est, th_rad, thd_rads])
            F_cart, _ = race.step(state, dt)

            # Convert force to drive percent
            drive_pct = float(np.clip(F_cart / MAX_CART_FORCE * 100.0, -100, 100))
            cmd_sock.sendto(
                f"DRIVE {drive_pct:.1f} {drive_pct:.1f}".encode(),
                (esp32_ip, CMD_PORT),
            )

            # Dead-reckoning update (Euler integration)
            a_est  = F_cart / CART_MASS
            xd_est = xd_est + a_est * dt
            x_est  = float(np.clip(x_est + xd_est * dt, -TRACK_HALF, TRACK_HALF))

            print(f"\r  x={x_est:+.2f}m  v={xd_est:+.2f}m/s  "
                  f"θ={np.degrees(th_rad):+.1f}°  "
                  f"F={drive_pct:+.0f}%  "
                  f"{'RECOVER' if race.is_recovering() else '       '}",
                  end="", flush=True)

            time.sleep(0.02)   # 50 Hz command rate

    except KeyboardInterrupt:
        print("\nAborted by user.")
    finally:
        cmd_sock.sendto(b"STOP", (esp32_ip, CMD_PORT))
        cmd_sock.close()
        tel_sock.close()
        print("\nStopped.")


# ─── Entry point ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Race Mode Controller")
    parser.add_argument("--live",  action="store_true",
                        help="Send commands to ESP32 (live hardware mode)")
    parser.add_argument("--esp32", default=ESP32_IP,
                        help=f"ESP32 IP address (default: {ESP32_IP})")
    parser.add_argument("--vmax",  type=float, default=VMAX_DEFAULT,
                        help=f"Cruise speed m/s (default: {VMAX_DEFAULT})")
    args = parser.parse_args()

    if args.live:
        run_live(args.esp32, args.vmax)
    else:
        run_simulation(args.vmax)


if __name__ == "__main__":
    main()
