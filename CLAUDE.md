# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Inverted Pendulum Race Car — a 4-wheeled cart that races a track while keeping an inverted pendulum upright via a reaction motor at the hinge.  The ESP32 reads a BNO055 IMU and runs LQR/PID control over a WiFi link from a PC.

## Commands

```bash
# Install Python dependencies
pip install -r requirements.txt

# Run all tests
python -m pytest simulation/tests/ -v

# Run a single test file
python -m pytest simulation/tests/test_physics.py -v

# Run a single test by name
python -m pytest simulation/tests/test_controllers.py::TestLQRDesign::test_closed_loop_is_stable -v

# Interactive pygame simulation (Modules 1-3)
python simulation/pendulum_sim.py

# Print LQR system info and gain values (copy these into lqr_controller.h)
python simulation/lqr_design.py

# GUI tuners (open separately while offline)
python tuner/pid_tuner_gui.py [--ip <ESP32_IP>]
python tuner/lqr_tuner_gui.py [--ip <ESP32_IP>]

# PC-side WiFi transmitter (Module 5)
python transmitter/transmitter.py [--ip <ESP32_IP>]
python transmitter/telemetry_dashboard.py

# System identification (Module 6)
python sysid/system_identification.py              # offline synthetic test
python sysid/system_identification.py --live --esp32 <IP>

# Race mode (Module 7)
python race_mode/race_controller.py                # simulation demo
python race_mode/race_controller.py --live --esp32 <IP> [--vmax 2.5]

# Flash ESP32 (requires PlatformIO)
cd esp32_firmware && pio run --target upload
pio device monitor
```

**Simulation keyboard controls:** `SPACE` start race, `LEFT/A` `RIGHT/D` drive, `P` toggle LQR↔PID, `R` reset, `Q/ESC` quit.

## Architecture

### Module map

| Module | Files | Role |
|--------|-------|------|
| 1 | `simulation/pendulum_sim.py` | Physics engine + pygame real-time GUI |
| 2 | `simulation/lqr_design.py` | LQR gain computation (Python) |
| 3 | `simulation/pid_controller.py` | PID + CascadePID (Python) |
| 4 | `esp32_firmware/` | Embedded control loop at 1 kHz |
| 5 | `transmitter/` | PC WiFi transmitter + telemetry dashboard |
| 6 | `sysid/system_identification.py` | Physical parameter fitting |
| 7 | `race_mode/race_controller.py` | Speed profile + feedforward + stability guard |
| — | `tuner/pid_tuner_gui.py` | tkinter/matplotlib PID slider tuner |
| — | `tuner/lqr_tuner_gui.py` | tkinter/matplotlib LQR weight tuner |

### Control flow in simulation (`pendulum_sim.py`)

Three sequential phases:
1. **READY** — pendulum horizontal, waiting for user.
2. **SWING-UP** — energy-pumping algorithm raises pendulum to vertical.
3. **RACE** — LQR or CascadePID stabilises pendulum; user drives cart.

`pendulum_sim.py` imports `lqr_design.py` and `pid_controller.py`.

### Physics model

- **State vector:** `[x, ẋ, θ, θ̇]` — cart position/velocity + pendulum angle/velocity.
- **Integration:** RK4 at 500 Hz (8 sub-steps per 60 FPS frame).
- **Actuators:** Reaction motor torque (pendulum) + cart force (driving).
- **Key parameters:** Cart 2.5 kg, pendulum 0.3 kg × 0.5 m rod, track ±4 m, max torque 5 N·m.
- **I_hinge** = 1/3 · 0.3 · 0.5² = 0.025 kg·m²

### LQR design (`lqr_design.py`)

- **Linearised 2-state model:** `[θ, θ̇]` — decouples reaction motor from cart.
- **Control law:** `τ = −K · [θ, θ̇]`
- Solves continuous-time ARE via `scipy.linalg.solve_continuous_are`.
- Default weights: `Q = diag(120, 20)`, `R = 0.01`.
- Run `python simulation/lqr_design.py` to get the K values to paste into `esp32_firmware/lqr_controller.h`.

### PID controllers (`pid_controller.py`)

- `PID` — single-loop with anti-windup integral clamp and derivative LP filter.
- `CascadePID` — outer angle loop → inner angular-velocity loop → torque.
- The C++ `pid_controller.cpp` is a direct port; gains transfer without rescaling.

### ESP32 firmware (`esp32_firmware/`)

- **1 kHz control loop** via `esp_timer` hardware timer (`control_loop_cb` in `main.cpp`).
- **Safety watchdog:** if BNO055 fails or `|θ| > 45°` → reaction motor stops immediately.
- **WiFi UDP protocol** (defined in `wifi_comm.h`):
  - PC → ESP32 port **4210**: text commands (`DRIVE`, `MODE`, `PID`, `LQR`, `TORQUE`, `STOP`).
  - ESP32 → PC port **4211**: JSON telemetry every 20 ms.
- **Build:** PlatformIO (`esp32_firmware/platformio.ini`). Libraries: Adafruit BNO055.
- **First-time setup:** Set `WIFI_SSID` and `WIFI_PASSWORD` in `main.cpp`; paste K gains from `lqr_design.py` into `lqr_controller.h`.

### Tuner GUIs (`tuner/`)

Both GUIs:
- Debounce slider changes to 150 ms, then re-run a quick closed-loop simulation.
- Show step-response plot (angle + torque for PID; angle + phase portrait for LQR).
- "Send to ESP32" button pushes gains over UDP.

`pid_tuner_gui.py` — 6 sliders (kp/ki/kd for outer and inner loops).
`lqr_tuner_gui.py` — 3 sliders (Q_theta, Q_omega, R_torque); also shows computed K and eigenvalues.

### WiFi protocol (shared between Python and C++)

Both `transmitter/transmitter.py` and `esp32_firmware/wifi_comm.cpp` implement the same protocol.  The tuner GUIs also use the same command format when uploading gains.

## Dependencies

```bash
pip install -r requirements.txt   # numpy, scipy, pygame, matplotlib, pytest
```

ESP32: PlatformIO fetches `Adafruit BNO055` and `Adafruit Unified Sensor` automatically via `platformio.ini`.
