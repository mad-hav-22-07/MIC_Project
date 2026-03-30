"""
pid_tuner_gui.py — Interactive PID Gains Tuner
================================================
A tkinter + matplotlib GUI for tuning the cascade PID gains offline
(against the Python simulation) and optionally pushing them to the ESP32.

Layout
------
  Left panel  : sliders for all 6 gains + performance metrics + buttons
  Right panel : embedded matplotlib plot (step response + torque output)

Workflow
--------
  1. Adjust sliders → step response updates automatically (< 150 ms lag).
  2. Watch settling time, peak overshoot, and steady-state error.
  3. Click "Send to ESP32" to push the gains over UDP to the running firmware.
  4. Click "Send to Simulation" to apply the gains in the pygame simulation
     (requires the simulation to be running on the same machine — not yet
     implemented; gains are printed to console for manual copy-paste).

Usage
-----
  python tuner/pid_tuner_gui.py [--ip <ESP32_IP>]
"""

import sys
import os
import argparse
import socket
import threading

import tkinter as tk
from tkinter import ttk

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Add simulation/ to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "simulation"))
from pid_controller import CascadePID
from pendulum_sim   import rk4_step, PHYSICS_DT, MAX_TORQUE

# ── Configuration ─────────────────────────────────────────────────────────────
ESP32_IP       = "192.168.1.100"
CMD_PORT       = 4210
SIM_SECONDS    = 6.0        # simulation horizon for step response
THETA0_DEG     = 8.0        # initial perturbation (degrees)
SETTLED_DEG    = 2.0        # threshold used for settling-time calculation
SETTLE_WINDOW  = 200        # steps that must stay settled (0.2 s at 500 Hz * ... no wait)
                             # actually at 500 Hz, 0.4 s worth of steps
POLL_MS        = 150        # replot debounce interval (ms)

# Default gains matching CascadePID defaults in pid_controller.py
GAIN_DEFAULTS = {
    "kp_outer": 20.0,
    "ki_outer":  0.0,
    "kd_outer":  0.0,
    "kp_inner":  1.5,
    "ki_inner":  0.0,
    "kd_inner":  0.0,
}

# Slider ranges: (min, max) for each gain
GAIN_RANGES = {
    "kp_outer": (0.0,  80.0),
    "ki_outer": (0.0,  20.0),
    "kd_outer": (0.0,  10.0),
    "kp_inner": (0.0,  15.0),
    "ki_inner": (0.0,  10.0),
    "kd_inner": (0.0,   5.0),
}

# Labels shown in the UI
GAIN_LABELS = {
    "kp_outer": "Kp  outer",
    "ki_outer": "Ki  outer",
    "kd_outer": "Kd  outer",
    "kp_inner": "Kp  inner",
    "ki_inner": "Ki  inner",
    "kd_inner": "Kd  inner",
}

# Dark colour palette
C_BG     = "#12151c"
C_PANEL  = "#1a1e28"
C_ACCENT = "#4682c8"
C_TEXT   = "#d0d5e0"
C_DIM    = "#6a6f80"
C_OK     = "#46c878"
C_WARN   = "#dc3c3c"


def _run_sim(gains: dict) -> tuple:
    """
    Run a closed-loop step-response simulation with the given PID gains.

    Returns
    -------
    (times, thetas_deg, taus) as numpy arrays, length = SIM_SECONDS / PHYSICS_DT
    """
    pid = CascadePID(
        kp_outer=gains["kp_outer"], ki_outer=gains["ki_outer"], kd_outer=gains["kd_outer"],
        kp_inner=gains["kp_inner"], ki_inner=gains["ki_inner"], kd_inner=gains["kd_inner"],
        max_output=MAX_TORQUE,
    )

    theta0 = np.radians(THETA0_DEG)
    state  = np.array([0.0, 0.0, theta0, 0.0])

    n_steps = int(SIM_SECONDS / PHYSICS_DT)
    times   = np.empty(n_steps)
    thetas  = np.empty(n_steps)
    taus    = np.empty(n_steps)

    t = 0.0
    for i in range(n_steps):
        tau       = pid.compute(state[2], state[3], PHYSICS_DT)
        state     = rk4_step(state, 0.0, tau)
        t        += PHYSICS_DT
        times[i]  = t
        thetas[i] = np.degrees(state[2])
        taus[i]   = tau

    return times, thetas, taus


def _compute_metrics(times: np.ndarray, thetas: np.ndarray) -> dict:
    """
    Extract performance metrics from a simulated step response.

    Returns dict with:
        settle_s   — settling time (s), or None if never settles
        overshoot  — peak absolute angle (degrees)
        ss_error   — mean absolute angle in last 0.5 s (degrees)
    """
    settled_mask = np.abs(thetas) < SETTLED_DEG

    settle_s = None
    # Find first index where the remaining trajectory stays settled
    for i in range(len(times)):
        window = settled_mask[i:]
        if len(window) >= SETTLE_WINDOW and window[:SETTLE_WINDOW].all():
            settle_s = float(times[i])
            break

    overshoot = float(np.max(np.abs(thetas)))
    ss_error  = float(np.mean(np.abs(thetas[-250:])))   # last 0.5 s

    return {"settle_s": settle_s, "overshoot": overshoot, "ss_error": ss_error}


class PIDTunerApp:
    """Main application window for the PID tuner."""

    def __init__(self, root: tk.Tk, esp32_ip: str):
        self.root      = root
        self.esp32_ip  = esp32_ip
        self._dirty    = False   # True when gains changed but plot not yet updated

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        root.title("PID Tuner — Inverted Pendulum Race Car")
        root.configure(bg=C_BG)
        root.resizable(True, True)

        self._build_ui()
        self._replot()   # initial render
        self._poll()     # start debounce loop

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        # ── Left panel: controls ─────────────────────────────────────────────
        left = tk.Frame(self.root, bg=C_PANEL, padx=12, pady=10)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(8, 4), pady=8)

        tk.Label(left, text="PID Tuner", font=("Consolas", 14, "bold"),
                 bg=C_PANEL, fg=C_TEXT).pack(anchor="w")
        tk.Label(left, text=f"Sim: {THETA0_DEG}° step, {SIM_SECONDS}s horizon",
                 font=("Consolas", 9), bg=C_PANEL, fg=C_DIM).pack(anchor="w")

        # Separator
        ttk.Separator(left, orient="horizontal").pack(fill=tk.X, pady=6)

        # ── Outer loop ───────────────────────────────────────────────────────
        tk.Label(left, text="Outer loop  (angle → ω setpoint)",
                 font=("Consolas", 9, "bold"), bg=C_PANEL, fg=C_ACCENT).pack(anchor="w", pady=(0,2))

        self._sliders = {}
        outer_keys = ["kp_outer", "ki_outer", "kd_outer"]
        inner_keys = ["kp_inner", "ki_inner", "kd_inner"]

        for key in outer_keys:
            self._add_slider(left, key)

        ttk.Separator(left, orient="horizontal").pack(fill=tk.X, pady=6)

        # ── Inner loop ───────────────────────────────────────────────────────
        tk.Label(left, text="Inner loop  (ω error → torque)",
                 font=("Consolas", 9, "bold"), bg=C_PANEL, fg=C_ACCENT).pack(anchor="w", pady=(0,2))

        for key in inner_keys:
            self._add_slider(left, key)

        ttk.Separator(left, orient="horizontal").pack(fill=tk.X, pady=8)

        # ── Performance metrics ───────────────────────────────────────────────
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

        # ── Right panel: matplotlib plot ──────────────────────────────────────
        right = tk.Frame(self.root, bg=C_BG)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(4, 8), pady=8)

        self._fig, (self._ax_th, self._ax_tau) = plt.subplots(
            2, 1, figsize=(7.5, 5.5), tight_layout=True
        )
        self._fig.patch.set_facecolor(C_BG)

        self._canvas = FigureCanvasTkAgg(self._fig, master=right)
        self._canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _add_slider(self, parent, key: str):
        """Add one labelled slider row to the parent frame."""
        lo, hi = GAIN_RANGES[key]
        label  = GAIN_LABELS[key]

        row  = tk.Frame(parent, bg=C_PANEL)
        row.pack(fill=tk.X, pady=2)

        tk.Label(row, text=f"{label}:", font=("Consolas", 9),
                 bg=C_PANEL, fg=C_DIM, width=12, anchor="w").pack(side=tk.LEFT)

        var = tk.DoubleVar(value=GAIN_DEFAULTS[key])
        # Mark dirty on any change (slider drag or direct set)
        var.trace_add("write", lambda *_: self._mark_dirty())

        scale = ttk.Scale(row, from_=lo, to=hi, orient=tk.HORIZONTAL,
                          variable=var, length=200)
        scale.pack(side=tk.LEFT, padx=4)

        val_lbl = tk.Label(row, text=f"{GAIN_DEFAULTS[key]:.3f}",
                           font=("Consolas", 9), bg=C_PANEL, fg=C_TEXT, width=7)
        val_lbl.pack(side=tk.LEFT)

        self._sliders[key] = (var, val_lbl)

    # ── Gain access ───────────────────────────────────────────────────────────

    def _get_gains(self) -> dict:
        return {k: v.get() for k, (v, _) in self._sliders.items()}

    # ── Debounce / replot ─────────────────────────────────────────────────────

    def _mark_dirty(self):
        """Called every time any slider moves.  Updates value labels."""
        for k, (var, lbl) in self._sliders.items():
            lbl.config(text=f"{var.get():.3f}")
        self._dirty = True

    def _poll(self):
        """Periodic callback: replot if gains changed since last render."""
        if self._dirty:
            self._dirty = False
            self._replot()
        self.root.after(POLL_MS, self._poll)

    def _replot(self):
        """Re-run the simulation with current gains and refresh the plots."""
        gains = self._get_gains()
        times, thetas, taus = _run_sim(gains)
        metrics = _compute_metrics(times, thetas)

        # Update metric labels
        st = metrics["settle_s"]
        self._metric_vars["Settling time (s)"].set(
            f"{st:.2f}" if st is not None else "unstable"
        )
        self._metric_vars["Peak overshoot (°)"].set(f"{metrics['overshoot']:.2f}")
        self._metric_vars["Steady-state (°)"].set(f"{metrics['ss_error']:.3f}")

        # ── Angle plot ────────────────────────────────────────────────────────
        ax = self._ax_th
        ax.clear()
        ax.set_facecolor("#161a22")
        ax.plot(times, thetas, color="cyan", linewidth=1.5, label="θ (°)")
        ax.axhline(0,            color="white",  lw=0.5, ls="--", alpha=0.6)
        ax.axhline( SETTLED_DEG, color="orange", lw=0.6, ls=":",  alpha=0.8)
        ax.axhline(-SETTLED_DEG, color="orange", lw=0.6, ls=":",  alpha=0.8,
                   label=f"±{SETTLED_DEG}° settled")
        ax.set_title(f"Step Response  (θ₀ = {THETA0_DEG}°)", color=C_TEXT, fontsize=9)
        ax.set_ylabel("Angle (°)", color="#aaaaaa", fontsize=8)
        ax.set_xlabel("Time (s)", color="#aaaaaa", fontsize=8)
        ax.tick_params(colors="#aaaaaa", labelsize=7)
        ax.legend(fontsize=7, facecolor="#1e2230", labelcolor="#cccccc",
                  loc="upper right")

        # Annotate settling time if available
        if st is not None:
            ax.axvline(st, color="lime", lw=0.8, ls="--", alpha=0.7)
            ax.text(st + 0.05, SETTLED_DEG * 1.5, f"t_s={st:.2f}s",
                    color="lime", fontsize=7)

        # ── Torque plot ───────────────────────────────────────────────────────
        ax2 = self._ax_tau
        ax2.clear()
        ax2.set_facecolor("#161a22")
        ax2.plot(times, taus, color="#e0c040", linewidth=1.2, label="τ (N·m)")
        ax2.axhline(0,           color="white",  lw=0.5, ls="--", alpha=0.6)
        ax2.axhline( MAX_TORQUE, color="#dc3c3c", lw=0.6, ls=":", alpha=0.7,
                     label=f"±{MAX_TORQUE} N·m limit")
        ax2.axhline(-MAX_TORQUE, color="#dc3c3c", lw=0.6, ls=":", alpha=0.7)
        ax2.set_title("Reaction Motor Torque", color=C_TEXT, fontsize=9)
        ax2.set_ylabel("τ (N·m)", color="#aaaaaa", fontsize=8)
        ax2.set_xlabel("Time (s)", color="#aaaaaa", fontsize=8)
        ax2.tick_params(colors="#aaaaaa", labelsize=7)
        ax2.legend(fontsize=7, facecolor="#1e2230", labelcolor="#cccccc",
                   loc="upper right")

        self._fig.patch.set_facecolor(C_BG)
        self._canvas.draw()

    # ── Button callbacks ──────────────────────────────────────────────────────

    def _send_to_esp32(self):
        """Send current PID gains to the ESP32 via UDP."""
        g = self._get_gains()
        msg = (f"PID {g['kp_outer']:.4f} {g['ki_outer']:.4f} {g['kd_outer']:.4f} "
               f"{g['kp_inner']:.4f} {g['ki_inner']:.4f} {g['kd_inner']:.4f}")
        try:
            self._sock.sendto(msg.encode(), (self.esp32_ip, CMD_PORT))
            self._status_var.set(f"✓ Sent → {self.esp32_ip}:{CMD_PORT}")
        except OSError as e:
            self._status_var.set(f"Send failed: {e}")

    def _reset_gains(self):
        """Reset all sliders to their default values."""
        for key, (var, _) in self._sliders.items():
            var.set(GAIN_DEFAULTS[key])
        self._status_var.set("Gains reset to defaults")


# ─── Entry point ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="PID Gains Tuner GUI")
    parser.add_argument("--ip", default=ESP32_IP,
                        help=f"ESP32 IP for gain upload (default: {ESP32_IP})")
    args = parser.parse_args()

    root = tk.Tk()
    root.configure(bg=C_BG)
    PIDTunerApp(root, args.ip)
    root.mainloop()


if __name__ == "__main__":
    main()
