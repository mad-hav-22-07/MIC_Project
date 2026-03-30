"""
telemetry_dashboard.py — Live Telemetry Dashboard (Module 5)
=============================================================
Receives JSON telemetry from the ESP32 over UDP and displays four
live scrolling plots:
  • Pendulum angle θ (degrees)
  • Angular velocity ω (degrees/s)
  • Reaction motor PWM (%)
  • Control loop timing (µs)  — useful for spotting timing jitter

Architecture
------------
  • A background daemon thread listens on UDP port 4211 and pushes
    decoded packets into a thread-safe ring buffer (TelemetryBuffer).
  • A matplotlib FuncAnimation updates the four axes every 50 ms,
    reading a snapshot from the buffer.
  • The main thread runs the matplotlib event loop (plt.show()).

Usage
-----
  python transmitter/telemetry_dashboard.py [--port PORT]

  Make sure the ESP32 is sending telemetry and your firewall allows
  incoming UDP on the telemetry port.
"""

import sys
import argparse
import socket
import json
import threading
from collections import deque

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── Settings ──────────────────────────────────────────────────────────────────
TEL_PORT      = 4211           # must match WIFI_TEL_PORT in wifi_comm.h
WINDOW_SECS   = 10.0           # seconds of data visible at once
ANIM_INTERVAL = 50             # matplotlib animation update (ms)
MAX_SAMPLES   = 5000           # ring-buffer depth


# ─── Thread-safe ring buffer ──────────────────────────────────────────────────

class TelemetryBuffer:
    """
    Double-buffered ring buffer for incoming telemetry packets.

    Writer thread: calls append() for each received packet.
    Reader (matplotlib): calls snapshot() to get a consistent copy.
    """

    def __init__(self, maxlen: int = MAX_SAMPLES):
        self._lock  = threading.Lock()
        self.t      = deque(maxlen=maxlen)   # time in seconds
        self.theta  = deque(maxlen=maxlen)   # angle (degrees)
        self.omega  = deque(maxlen=maxlen)   # angular velocity (degrees/s)
        self.pwm    = deque(maxlen=maxlen)   # PWM percent
        self.dt_us  = deque(maxlen=maxlen)   # loop timing (µs)
        self.n_recv = 0                       # total packets received

    def append(self, pkt: dict):
        with self._lock:
            # Convert ESP32 millis → seconds
            self.t.append(pkt.get("t", 0) / 1000.0)
            self.theta.append(pkt.get("th",     0.0))
            self.omega.append(pkt.get("om",     0.0))
            self.pwm.append(  pkt.get("pwm",    0.0))
            self.dt_us.append(pkt.get("dt_us",  0))
            self.n_recv += 1

    def snapshot(self):
        """Return a consistent (t, theta, omega, pwm, dt_us) tuple of lists."""
        with self._lock:
            return (list(self.t), list(self.theta),
                    list(self.omega), list(self.pwm), list(self.dt_us))

    def packet_count(self) -> int:
        with self._lock:
            return self.n_recv


# ─── Background receive thread ────────────────────────────────────────────────

def _receive_loop(buf: TelemetryBuffer, port: int, stop: threading.Event):
    """
    Listen on UDP port and push decoded packets into the ring buffer.
    Runs as a daemon thread — exits when the main process ends or stop is set.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.5)   # periodic check for stop flag

    print(f"Telemetry dashboard: listening on UDP port {port}")

    while not stop.is_set():
        try:
            data, addr = sock.recvfrom(512)
            pkt = json.loads(data.decode())
            buf.append(pkt)
        except socket.timeout:
            pass
        except json.JSONDecodeError:
            pass   # malformed packet — ignore

    sock.close()


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Inverted Pendulum Telemetry Dashboard")
    parser.add_argument("--port", type=int, default=TEL_PORT,
                        help=f"UDP port for telemetry (default: {TEL_PORT})")
    args = parser.parse_args()

    buf  = TelemetryBuffer()
    stop = threading.Event()

    recv_thread = threading.Thread(
        target=_receive_loop,
        args=(buf, args.port, stop),
        daemon=True,
        name="telemetry-recv",
    )
    recv_thread.start()

    # ── Figure layout ─────────────────────────────────────────────────────────
    fig, axes = plt.subplots(4, 1, figsize=(11, 8), sharex=True)
    fig.patch.set_facecolor("#12151c")
    plt.subplots_adjust(left=0.09, right=0.97, top=0.93, bottom=0.07, hspace=0.35)

    plot_specs = [
        # (title,                  ylabel,        colour,   y_lo,   y_hi,  hline)
        ("Pendulum Angle",         "θ (°)",        "cyan",   -60,     60,    0),
        ("Angular Velocity",       "ω (°/s)",      "orange", -400,   400,    0),
        ("Reaction Motor PWM",     "PWM (%)",      "lime",   -110,   110,    0),
        ("Control Loop Timing",    "Δt (µs)",      "violet",    0,  5000, 1000),
    ]

    lines_list = []
    for ax, (title, ylabel, colour, ylo, yhi, hl) in zip(axes, plot_specs):
        ax.set_facecolor("#161a22")
        ax.set_title(title, color="#ccccdd", fontsize=9, pad=3)
        ax.set_ylabel(ylabel, color="#aaaaaa", fontsize=8)
        ax.set_ylim(ylo, yhi)
        ax.axhline(hl, color="white", linewidth=0.5, linestyle="--", alpha=0.4)
        ax.tick_params(colors="#888899", labelsize=7)
        for spine in ax.spines.values():
            spine.set_color("#282838")

        lp, = ax.plot([], [], colour, linewidth=1.3)
        lines_list.append(lp)

    # Stability margin reference lines on the angle plot
    axes[0].axhline( 20, color="#ff6060", linewidth=0.6, linestyle=":", alpha=0.7,
                     label="±20° warn")
    axes[0].axhline(-20, color="#ff6060", linewidth=0.6, linestyle=":", alpha=0.7)
    axes[0].legend(fontsize=7, facecolor="#1e2230", labelcolor="#aaaaaa",
                   loc="upper right")

    # 1 kHz reference on loop timing plot
    axes[3].axhline(1000, color="yellow", linewidth=0.5, linestyle="--", alpha=0.5)

    axes[-1].set_xlabel("Time (s)", color="#aaaaaa", fontsize=8)

    status_text = axes[0].text(
        0.01, 0.80, "Waiting for data...",
        transform=axes[0].transAxes,
        color="gray", fontsize=7, va="top",
    )

    plt.suptitle("Inverted Pendulum Race Car — Live Telemetry",
                 color="#dddddd", fontsize=11, y=0.98)

    # ── Animation callback ────────────────────────────────────────────────────

    data_arrays = [None, None, None, None]   # populated in update_plot

    def update_plot(frame):
        t_all, theta, omega, pwm, dt_us = buf.snapshot()
        if not t_all:
            return lines_list

        t_arr     = np.array(t_all)
        now       = t_arr[-1]
        mask      = t_arr >= now - WINDOW_SECS

        data_arrays[0] = np.array(theta)
        data_arrays[1] = np.array(omega)
        data_arrays[2] = np.array(pwm)
        data_arrays[3] = np.array(dt_us)

        for lp, data in zip(lines_list, data_arrays):
            if data is not None:
                lp.set_data(t_arr[mask], data[mask])
            lp.axes.set_xlim(now - WINDOW_SECS, now + 0.2)

        n       = int(mask.sum())
        rate_hz = n / WINDOW_SECS
        status_text.set_text(
            f"{buf.packet_count()} pkts recv  |  {rate_hz:.0f} Hz  |  t = {now:.1f} s"
        )
        status_text.set_color("#aaaaaa")

        return lines_list

    ani = animation.FuncAnimation(
        fig, update_plot, interval=ANIM_INTERVAL, blit=True
    )

    # ── Show (blocks until window is closed) ─────────────────────────────────
    try:
        plt.show()
    finally:
        stop.set()
        recv_thread.join(timeout=2.0)
        print("Dashboard closed.")


if __name__ == "__main__":
    main()
