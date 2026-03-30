"""
lqr_tuner_gui.py — Interactive LQR Weight Tuner
=================================================
A tkinter + matplotlib GUI for choosing the LQR Q/R weights, viewing the
resulting gain vector and closed-loop stability, and uploading gains to
the ESP32 over WiFi.

LQR recap
---------
The 2-state LQR minimises  ∫(x^T Q x + u^T R u) dt  subject to the
linearised pendulum dynamics.  Increasing Q[0,0] (angle weight) makes
the controller prioritise keeping θ small.  Increasing Q[1,1] (velocity
weight) damps oscillations faster.  Decreasing R makes the controller use
more motor torque.

Layout
------
  Left panel  : three sliders (Q_theta, Q_omega, R_torque)
                + live readout of K gains and eigenvalues
                + buttons
  Right panel : embedded matplotlib plot (step response + phase portrait)

Usage
-----
  python tuner/lqr_tuner_gui.py [--ip <ESP32_IP>]
"""

import sys
import os
import argparse
import socket

import tkinter as tk
from tkinter import ttk

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "simulation"))
from lqr_design  import compute_lqr_gains, get_system_matrices
from pendulum_sim import rk4_step, PHYSICS_DT, MAX_TORQUE

# ── Configuration ─────────────────────────────────────────────────────────────
ESP32_IP     = "192.168.1.100"
CMD_PORT     = 4210
SIM_SECONDS  = 6.0
THETA0_DEG   = 8.0
POLL_MS      = 150

# Default Q/R weights (must match lqr_design.py defaults)
DEFAULTS = {"Q_theta": 120.0, "Q_omega": 20.0, "R_torque": 0.01}

# Slider ranges
RANGES = {
    "Q_theta": (1.0,   500.0),
    "Q_omega": (1.0,   100.0),
    "R_torque":(0.001,   0.5),
}

# Colour palette
C_BG    = "#12151c"
C_PANEL = "#1a1e28"
C_TEXT  = "#d0d5e0"
C_DIM   = "#6a6f80"
C_ACC   = "#4682c8"
C_OK    = "#46c878"
C_WARN  = "#dc3c3c"


def _build_qr(Q_theta: float, Q_omega: float, R_torque: float) -> tuple:
    """Construct Q and R matrices from scalar slider values."""
    Q = np.diag([Q_theta, Q_omega])
    R = np.array([[R_torque]])
    return Q, R


def _run_sim(K: np.ndarray) -> tuple:
    """
    Closed-loop step response with LQR gain K.
    Returns (times, thetas_deg, taus) as numpy arrays.
    """
    theta0 = np.radians(THETA0_DEG)
    state  = np.array([0.0, 0.0, theta0, 0.0])

    n = int(SIM_SECONDS / PHYSICS_DT)
    times  = np.empty(n)
    thetas = np.empty(n)
    taus   = np.empty(n)

    t = 0.0
    for i in range(n):
        tau      = float(np.clip(-K @ state[2:4], -MAX_TORQUE, MAX_TORQUE))
        state    = rk4_step(state, 0.0, tau)
        t       += PHYSICS_DT
        times[i] = t
        thetas[i]= np.degrees(state[2])
        taus[i]  = tau

    return times, thetas, taus


def _settle_time(times, thetas, threshold_deg=2.0, window=200) -> float | None:
    """First time the trajectory stays within ±threshold_deg for <window> steps."""
    mask = np.abs(thetas) < threshold_deg
    for i in range(len(times)):
        if mask[i:i+window].all():
            return float(times[i])
    return None


class LQRTunerApp:
    """Main application window for the LQR weight tuner."""

    def __init__(self, root: tk.Tk, esp32_ip: str):
        self.root     = root
        self.esp32_ip = esp32_ip
        self._dirty   = False
        self._sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        root.title("LQR Tuner — Inverted Pendulum Race Car")
        root.configure(bg=C_BG)
        root.resizable(True, True)

        self._build_ui()
        self._replot()
        self._poll()

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        # Left panel
        left = tk.Frame(self.root, bg=C_PANEL, padx=12, pady=10)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(8, 4), pady=8)

        tk.Label(left, text="LQR Tuner", font=("Consolas", 14, "bold"),
                 bg=C_PANEL, fg=C_TEXT).pack(anchor="w")
        tk.Label(left, text="Adjust Q/R weights → see K gains + step response",
                 font=("Consolas", 9), bg=C_PANEL, fg=C_DIM).pack(anchor="w")

        ttk.Separator(left, orient="horizontal").pack(fill=tk.X, pady=6)

        # ── Q/R sliders ───────────────────────────────────────────────────────
        tk.Label(left, text="Cost matrices",
                 font=("Consolas", 9, "bold"), bg=C_PANEL, fg=C_ACC).pack(anchor="w")

        descs = {
            "Q_theta":  "Angle weight   (↑ = tighter angle control)",
            "Q_omega":  "Velocity weight(↑ = faster damping)",
            "R_torque": "Torque penalty (↓ = more aggressive)",
        }

        self._sliders = {}
        for key in ("Q_theta", "Q_omega", "R_torque"):
            self._add_slider(left, key, descs[key])

        ttk.Separator(left, orient="horizontal").pack(fill=tk.X, pady=8)

        # ── Computed gains readout ────────────────────────────────────────────
        tk.Label(left, text="Computed LQR gains",
                 font=("Consolas", 9, "bold"), bg=C_PANEL, fg=C_TEXT).pack(anchor="w")

        self._gain_vars = {}
        for label in ("K₀  (angle gain)", "K₁  (velocity gain)"):
            row = tk.Frame(left, bg=C_PANEL)
            row.pack(fill=tk.X, pady=1)
            tk.Label(row, text=f"{label}:", font=("Consolas", 9),
                     bg=C_PANEL, fg=C_DIM, width=18, anchor="w").pack(side=tk.LEFT)
            v = tk.StringVar(value="—")
            tk.Label(row, textvariable=v, font=("Consolas", 9, "bold"),
                     bg=C_PANEL, fg=C_TEXT, width=10).pack(side=tk.LEFT)
            self._gain_vars[label] = v

        ttk.Separator(left, orient="horizontal").pack(fill=tk.X, pady=6)

        # ── Eigenvalue readout ────────────────────────────────────────────────
        tk.Label(left, text="Closed-loop eigenvalues",
                 font=("Consolas", 9, "bold"), bg=C_PANEL, fg=C_TEXT).pack(anchor="w")

        self._eig_vars = {}
        for label in ("λ₁ (real)", "λ₂ (real)", "Stable?"):
            row = tk.Frame(left, bg=C_PANEL)
            row.pack(fill=tk.X, pady=1)
            tk.Label(row, text=f"{label}:", font=("Consolas", 9),
                     bg=C_PANEL, fg=C_DIM, width=18, anchor="w").pack(side=tk.LEFT)
            v = tk.StringVar(value="—")
            col_var = tk.StringVar(value=C_TEXT)
            lbl = tk.Label(row, textvariable=v, font=("Consolas", 9, "bold"),
                           bg=C_PANEL, fg=C_TEXT, width=10)
            lbl.pack(side=tk.LEFT)
            self._eig_vars[label] = (v, lbl)

        ttk.Separator(left, orient="horizontal").pack(fill=tk.X, pady=8)

        # ── Performance ───────────────────────────────────────────────────────
        tk.Label(left, text="Performance",
                 font=("Consolas", 10, "bold"), bg=C_PANEL, fg=C_TEXT).pack(anchor="w")

        self._metric_vars = {}
        for name in ("Settling time (s)", "Peak overshoot (°)", "Steady-state (°)"):
            row = tk.Frame(left, bg=C_PANEL)
            row.pack(fill=tk.X, pady=1)
            tk.Label(row, text=f"{name}:", font=("Consolas", 9),
                     bg=C_PANEL, fg=C_DIM, width=20, anchor="w").pack(side=tk.LEFT)
            v = tk.StringVar(value="—")
            tk.Label(row, textvariable=v, font=("Consolas", 9, "bold"),
                     bg=C_PANEL, fg=C_TEXT, width=8, anchor="e").pack(side=tk.LEFT)
            self._metric_vars[name] = v

        ttk.Separator(left, orient="horizontal").pack(fill=tk.X, pady=8)

        # ── Buttons ───────────────────────────────────────────────────────────
        btn_frame = tk.Frame(left, bg=C_PANEL)
        btn_frame.pack()

        tk.Button(btn_frame, text="Send to ESP32",
                  command=self._send_to_esp32,
                  bg="#2a4a80", fg="white", font=("Consolas", 10),
                  relief="flat", padx=8, pady=4).pack(side=tk.LEFT, padx=4)

        tk.Button(btn_frame, text="Reset defaults",
                  command=self._reset_gains,
                  bg="#3a3f4e", fg=C_TEXT, font=("Consolas", 10),
                  relief="flat", padx=8, pady=4).pack(side=tk.LEFT, padx=4)

        self._status_var = tk.StringVar(value="Ready")
        tk.Label(left, textvariable=self._status_var,
                 font=("Consolas", 9), bg=C_PANEL, fg=C_DIM).pack(pady=(6, 0))

        # Right panel: matplotlib
        right = tk.Frame(self.root, bg=C_BG)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(4, 8), pady=8)

        self._fig, (self._ax_th, self._ax_pp) = plt.subplots(
            2, 1, figsize=(7.5, 5.5), tight_layout=True
        )
        self._fig.patch.set_facecolor(C_BG)

        self._canvas = FigureCanvasTkAgg(self._fig, master=right)
        self._canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _add_slider(self, parent, key: str, desc: str):
        lo, hi = RANGES[key]
        frame  = tk.Frame(parent, bg=C_PANEL)
        frame.pack(fill=tk.X, pady=3)

        tk.Label(frame, text=desc, font=("Consolas", 8),
                 bg=C_PANEL, fg=C_DIM).pack(anchor="w")

        row = tk.Frame(frame, bg=C_PANEL)
        row.pack(fill=tk.X)

        tk.Label(row, text=f"{key}:", font=("Consolas", 9),
                 bg=C_PANEL, fg=C_ACC, width=9, anchor="w").pack(side=tk.LEFT)

        var = tk.DoubleVar(value=DEFAULTS[key])
        var.trace_add("write", lambda *_: self._mark_dirty())

        scale = ttk.Scale(row, from_=lo, to=hi, orient=tk.HORIZONTAL,
                          variable=var, length=190)
        scale.pack(side=tk.LEFT, padx=4)

        val_lbl = tk.Label(row, text=f"{DEFAULTS[key]:.4f}",
                           font=("Consolas", 9), bg=C_PANEL, fg=C_TEXT, width=8)
        val_lbl.pack(side=tk.LEFT)

        self._sliders[key] = (var, val_lbl)

    # ── Logic ─────────────────────────────────────────────────────────────────

    def _get_values(self) -> dict:
        return {k: v.get() for k, (v, _) in self._sliders.items()}

    def _mark_dirty(self):
        for k, (var, lbl) in self._sliders.items():
            lbl.config(text=f"{var.get():.4f}")
        self._dirty = True

    def _poll(self):
        if self._dirty:
            self._dirty = False
            self._replot()
        self.root.after(POLL_MS, self._poll)

    def _replot(self):
        vals = self._get_values()
        Q, R = _build_qr(vals["Q_theta"], vals["Q_omega"], vals["R_torque"])

        # Try to compute LQR gains (may fail if R is very large or Q is degenerate)
        try:
            K = compute_lqr_gains(Q, R)
        except Exception as e:
            self._status_var.set(f"LQR solver error: {e}")
            return

        # Update gain readout
        gain_labels = list(self._gain_vars.keys())
        for i, label in enumerate(gain_labels):
            self._gain_vars[label].set(f"{K[i]:.4f}")

        # Closed-loop eigenvalues
        A, B = get_system_matrices()
        A_cl  = A - B @ K.reshape(1, -1)
        eigs  = np.linalg.eigvals(A_cl)
        stable = all(e.real < 0 for e in eigs)

        eig_labels = list(self._eig_vars.keys())
        for i, label in enumerate(eig_labels[:2]):
            v, lbl = self._eig_vars[label]
            e_real = float(eigs[i].real)
            v.set(f"{e_real:.4f}")
            lbl.config(fg=C_OK if e_real < 0 else C_WARN)

        sv, sl = self._eig_vars["Stable?"]
        sv.set("YES" if stable else "NO")
        sl.config(fg=C_OK if stable else C_WARN)

        if not stable:
            self._status_var.set("WARNING: closed loop is UNSTABLE with these weights")
            # still plot to show the diverging response
        else:
            self._status_var.set(f"K = [{K[0]:.3f}, {K[1]:.3f}]")

        # Run simulation
        times, thetas, taus = _run_sim(K)
        st = _settle_time(times, thetas)

        # Performance metrics
        self._metric_vars["Settling time (s)"].set(
            f"{st:.2f}" if st is not None else "unstable"
        )
        self._metric_vars["Peak overshoot (°)"].set(f"{np.max(np.abs(thetas)):.2f}")
        self._metric_vars["Steady-state (°)"].set(f"{np.mean(np.abs(thetas[-250:])):.3f}")

        # ── Angle plot ────────────────────────────────────────────────────────
        ax = self._ax_th
        ax.clear()
        ax.set_facecolor("#161a22")
        ax.plot(times, thetas, color="cyan", lw=1.5, label="θ (°)")
        ax.axhline(0,    color="white",  lw=0.5, ls="--", alpha=0.6)
        ax.axhline( 2.0, color="orange", lw=0.6, ls=":",  alpha=0.8)
        ax.axhline(-2.0, color="orange", lw=0.6, ls=":",  alpha=0.8, label="±2° settled")
        if st is not None:
            ax.axvline(st, color="lime", lw=0.8, ls="--", alpha=0.7)
            ax.text(st + 0.05, 2.5, f"t_s={st:.2f}s", color="lime", fontsize=7)
        ax.set_title(f"Step Response  (θ₀ = {THETA0_DEG}°)  "
                     f"K = [{K[0]:.2f}, {K[1]:.2f}]", color=C_TEXT, fontsize=9)
        ax.set_ylabel("Angle (°)", color="#aaaaaa", fontsize=8)
        ax.set_xlabel("Time (s)", color="#aaaaaa", fontsize=8)
        ax.tick_params(colors="#aaaaaa", labelsize=7)
        ax.legend(fontsize=7, facecolor="#1e2230", labelcolor="#cccccc", loc="upper right")

        # ── Phase portrait: θ vs θ̇ ─────────────────────────────────────────
        # Shows the trajectory in state space — a good spiral → stable
        ax2 = self._ax_pp
        ax2.clear()
        ax2.set_facecolor("#161a22")

        # We need theta_dot as well — rerun to collect it
        theta0 = np.radians(THETA0_DEG)
        state  = np.array([0.0, 0.0, theta0, 0.0])
        pp_th  = [np.degrees(state[2])]
        pp_thd = [np.degrees(state[3])]
        for _ in range(int(SIM_SECONDS / PHYSICS_DT)):
            tau   = float(np.clip(-K @ state[2:4], -MAX_TORQUE, MAX_TORQUE))
            state = rk4_step(state, 0.0, tau)
            pp_th.append(np.degrees(state[2]))
            pp_thd.append(np.degrees(state[3]))

        pp_th  = np.array(pp_th)
        pp_thd = np.array(pp_thd)

        # Colour gradient from start (bright) to end (dim)
        n_pts = len(pp_th)
        for j in range(0, n_pts - 1, max(1, n_pts // 200)):
            alpha = 0.3 + 0.7 * (1.0 - j / n_pts)
            ax2.plot(pp_th[j:j+2], pp_thd[j:j+2], color="cyan",
                     lw=1.0, alpha=alpha)

        ax2.plot(pp_th[0], pp_thd[0], "o", color="lime",  ms=5, label="start")
        ax2.plot(pp_th[-1], pp_thd[-1], "x", color="red", ms=5, label="end")
        ax2.axhline(0, color="white", lw=0.4, ls="--", alpha=0.4)
        ax2.axvline(0, color="white", lw=0.4, ls="--", alpha=0.4)
        ax2.set_title("Phase Portrait (θ vs θ̇)", color=C_TEXT, fontsize=9)
        ax2.set_xlabel("θ (°)", color="#aaaaaa", fontsize=8)
        ax2.set_ylabel("θ̇ (°/s)", color="#aaaaaa", fontsize=8)
        ax2.tick_params(colors="#aaaaaa", labelsize=7)
        ax2.legend(fontsize=7, facecolor="#1e2230", labelcolor="#cccccc", loc="upper right")

        self._fig.patch.set_facecolor(C_BG)
        self._canvas.draw()

    # ── Buttons ───────────────────────────────────────────────────────────────

    def _send_to_esp32(self):
        """Compute K from current Q/R and upload to ESP32."""
        vals = self._get_values()
        Q, R = _build_qr(vals["Q_theta"], vals["Q_omega"], vals["R_torque"])
        try:
            K = compute_lqr_gains(Q, R)
        except Exception as e:
            self._status_var.set(f"LQR solver error: {e}")
            return
        msg = f"LQR {K[0]:.4f} {K[1]:.4f}"
        try:
            self._sock.sendto(msg.encode(), (self.esp32_ip, CMD_PORT))
            self._status_var.set(f"✓ Sent K=[{K[0]:.3f},{K[1]:.3f}] → {self.esp32_ip}")
        except OSError as e:
            self._status_var.set(f"Send failed: {e}")

    def _reset_gains(self):
        for key, (var, _) in self._sliders.items():
            var.set(DEFAULTS[key])
        self._status_var.set("Weights reset to defaults")


# ─── Entry point ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="LQR Weight Tuner GUI")
    parser.add_argument("--ip", default=ESP32_IP,
                        help=f"ESP32 IP for gain upload (default: {ESP32_IP})")
    args = parser.parse_args()

    root = tk.Tk()
    root.configure(bg=C_BG)
    LQRTunerApp(root, args.ip)
    root.mainloop()


if __name__ == "__main__":
    main()
