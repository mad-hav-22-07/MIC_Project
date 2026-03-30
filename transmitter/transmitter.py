"""
transmitter.py — WiFi Transmitter (Module 5)
=============================================
PC-side keyboard control and WiFi link to the ESP32.

Architecture
------------
  • Uses pygame for the window and held-key detection (60 Hz).
  • Sends wheel drive commands over UDP every frame.
  • Receives JSON telemetry from the ESP32 and displays it in real time.
  • All commands use the text protocol defined in wifi_comm.h.

Keyboard controls
-----------------
  LEFT / A        : drive left
  RIGHT / D       : drive right
  P               : toggle LQR ↔ PID mode on ESP32
  S               : emergency stop (sends STOP, zeros wheels)
  L               : push default LQR gains (computed locally by lqr_design.py)
  + / =           : increase wheel drive speed
  -               : decrease wheel drive speed
  Q / Esc         : quit

Usage
-----
  1. Set ESP32_IP below (or pass --ip on the command line).
  2. Make sure the ESP32 is running main.cpp and connected to the same WiFi network.
  3. Run:  python transmitter/transmitter.py
"""

import sys
import os
import argparse
import socket
import json
import threading

import pygame

# Add simulation/ to path so we can call compute_lqr_gains()
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "simulation"))
from lqr_design import compute_lqr_gains

# ── Network defaults (override via --ip CLI argument) ────────────────────────
ESP32_IP   = "192.168.1.100"   # change to your ESP32's IP address
CMD_PORT   = 4210               # ESP32 listens on this port
TEL_PORT   = 4211               # ESP32 sends telemetry to this port

# ── Drive speed settings ──────────────────────────────────────────────────────
DRIVE_STEP     = 10     # speed change per +/- key press (percent)
DRIVE_MIN      = 10
DRIVE_MAX      = 100

SCREEN_W       = 860
SCREEN_H       = 420

# Colour palette
C_BG    = (22,  26,  34)
C_HUD   = (210, 215, 225)
C_OK    = (70,  200, 120)
C_WARN  = (220, 60,  60)
C_DIM   = (110, 115, 135)
C_LQR   = (70,  200, 120)
C_PID   = (210, 155, 70)


class Transmitter:
    """
    Manages the UDP sockets and last-known telemetry for the transmitter GUI.

    Methods called from the main thread:
      send(msg)            — fire-and-forget UDP command
      send_drive(l, r)     — DRIVE command
      send_mode(m)         — MODE command
      send_lqr_gains(K)    — LQR command
      send_pid_gains(...)  — PID command
      poll_telemetry()     — non-blocking receive (call every frame)
      get_telemetry()      — thread-safe snapshot of last received packet
    """

    def __init__(self, esp32_ip: str):
        self.esp32_ip    = esp32_ip
        self.mode        = "LQR"     # tracked locally for display
        self.drive_speed = 50        # current drive power (percent)

        # Outgoing command socket (fire-and-forget UDP)
        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Incoming telemetry socket (non-blocking)
        self._tel_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._tel_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._tel_sock.bind(("0.0.0.0", TEL_PORT))
        self._tel_sock.settimeout(0.0)   # fully non-blocking

        self._telem       = {}
        self._telem_lock  = threading.Lock()

    # ── Send helpers ──────────────────────────────────────────────────────────

    def send(self, msg: str):
        """Send a raw command string to the ESP32 (UDP, no acknowledgement)."""
        try:
            self._cmd_sock.sendto(msg.encode(), (self.esp32_ip, CMD_PORT))
        except OSError:
            pass   # silently ignore send failures (ESP32 may be unreachable)

    def send_drive(self, left: float, right: float):
        self.send(f"DRIVE {left:.1f} {right:.1f}")

    def send_mode(self, mode: str):
        self.mode = mode
        self.send(f"MODE {mode}")

    def send_stop(self):
        self.send("STOP")

    def send_lqr_gains(self, K0: float, K1: float):
        self.send(f"LQR {K0:.4f} {K1:.4f}")

    def send_pid_gains(self, kpo, kio, kdo, kpi, kii, kdi):
        self.send(f"PID {kpo:.4f} {kio:.4f} {kdo:.4f} {kpi:.4f} {kii:.4f} {kdi:.4f}")

    # ── Receive ───────────────────────────────────────────────────────────────

    def poll_telemetry(self):
        """
        Non-blocking receive of one telemetry packet.
        Call once per display frame.  Silently ignores empty/malformed packets.
        """
        try:
            data, _ = self._tel_sock.recvfrom(512)
            telem = json.loads(data.decode())
            with self._telem_lock:
                self._telem = telem
        except (BlockingIOError, socket.timeout, json.JSONDecodeError, OSError):
            pass

    def get_telemetry(self) -> dict:
        """Thread-safe snapshot of the most recent telemetry packet."""
        with self._telem_lock:
            return dict(self._telem)


def main():
    parser = argparse.ArgumentParser(description="Inverted Pendulum Race Car Transmitter")
    parser.add_argument("--ip", default=ESP32_IP, help="ESP32 IP address")
    args = parser.parse_args()

    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Inverted Pendulum — Transmitter")
    clock  = pygame.time.Clock()
    font_l = pygame.font.SysFont("Consolas", 22, bold=True)
    font_m = pygame.font.SysFont("Consolas", 19)
    font_s = pygame.font.SysFont("Consolas", 16)

    tx = Transmitter(args.ip)
    print(f"Transmitter ready.  Sending commands → {args.ip}:{CMD_PORT}")
    print(f"Receiving telemetry on UDP port {TEL_PORT}")

    running = True
    while running:

        # ── Event handling ────────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False

                elif event.key == pygame.K_p:
                    # Toggle stabiliser mode on the ESP32
                    new_mode = "PID" if tx.mode == "LQR" else "LQR"
                    tx.send_mode(new_mode)

                elif event.key == pygame.K_s:
                    tx.send_stop()

                elif event.key in (pygame.K_EQUALS, pygame.K_PLUS):
                    tx.drive_speed = min(DRIVE_MAX, tx.drive_speed + DRIVE_STEP)

                elif event.key == pygame.K_MINUS:
                    tx.drive_speed = max(DRIVE_MIN, tx.drive_speed - DRIVE_STEP)

                elif event.key == pygame.K_l:
                    # Recompute and push LQR gains from the local Python model
                    K = compute_lqr_gains()
                    tx.send_lqr_gains(K[0], K[1])
                    print(f"Pushed LQR gains: K=[{K[0]:.2f}, {K[1]:.2f}]")

        # ── Held-key drive commands ───────────────────────────────────────────
        keys   = pygame.key.get_pressed()
        drive_l = 0.0
        drive_r = 0.0

        if keys[pygame.K_RIGHT] or keys[pygame.K_d]:
            drive_l = drive_r = float(tx.drive_speed)
        if keys[pygame.K_LEFT] or keys[pygame.K_a]:
            drive_l = drive_r = -float(tx.drive_speed)

        tx.send_drive(drive_l, drive_r)
        tx.poll_telemetry()
        telem = tx.get_telemetry()

        # ── Render ────────────────────────────────────────────────────────────
        screen.fill(C_BG)

        # Title
        screen.blit(font_l.render("Inverted Pendulum Race Car — Transmitter", True, C_HUD),
                    (20, 14))

        # Connection / data status
        live     = bool(telem)
        con_col  = C_OK if live else C_WARN
        con_txt  = f"ESP32  {args.ip}  {'● LIVE' if live else '○ NO DATA'}"
        screen.blit(font_s.render(con_txt, True, con_col), (20, 52))

        # Drive bar
        bar_w = 240
        pct   = abs(drive_l) / 100.0
        bar_col = C_OK if drive_l != 0 else C_DIM
        pygame.draw.rect(screen, (40, 45, 55), (20, 78, bar_w, 14), border_radius=3)
        if pct > 0:
            pygame.draw.rect(screen, bar_col, (20, 78, int(bar_w * pct), 14), border_radius=3)
        dir_str = "►" if drive_l > 0 else ("◄" if drive_l < 0 else "■")
        screen.blit(font_m.render(f"Drive {tx.drive_speed}%  {dir_str}", True, C_HUD), (270, 74))

        # Mode
        m_col = C_LQR if tx.mode == "LQR" else C_PID
        screen.blit(font_m.render(f"Mode : {tx.mode}", True, m_col), (20, 106))

        # Telemetry panel
        if telem:
            th   = telem.get("th", 0.0)
            om   = telem.get("om", 0.0)
            pwm  = telem.get("pwm", 0.0)
            dt   = telem.get("dt_us", 0)
            mode = telem.get("mode", "—")

            th_col  = C_WARN if abs(th) > 20 else C_HUD
            pwm_col = C_OK if pwm >= 0 else C_WARN

            rows = [
                ("Angle θ",   f"{th:+7.2f} °",      th_col),
                ("ω",         f"{om:+7.1f} °/s",     C_HUD),
                ("Motor PWM", f"{pwm:+7.1f} %",      pwm_col),
                ("Loop Δt",   f"{dt:6d} µs",         C_HUD),
                ("ESP32 mode",f"{mode}",              m_col),
            ]
            for i, (label, val, col) in enumerate(rows):
                screen.blit(font_m.render(f"{label:<12}: {val}", True, col), (20, 150 + i * 28))

        else:
            screen.blit(font_m.render("No telemetry received yet...", True, C_DIM), (20, 150))

        # Key legend
        legend = [
            "←/→  or  A/D  : drive          P  : toggle LQR / PID",
            "S             : stop           +/- : adjust speed",
            "L             : push LQR gains  Q  : quit",
        ]
        for i, line in enumerate(legend):
            screen.blit(font_s.render(line, True, C_DIM), (20, SCREEN_H - 82 + i * 24))

        pygame.display.flip()
        clock.tick(60)

    tx.send_stop()
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
