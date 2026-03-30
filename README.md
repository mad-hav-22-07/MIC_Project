# Inverted Pendulum Race Car — Course Project

## System Overview

```
[WiFi Transmitter PC] <--WiFi--> [ESP32 on Cart]
                                       |
                              [4x Wheel Motors (PWM)]
                              [Reaction Motor (PWM) — DC + Gearbox, direct ESP32 GPIO]
                              [BNO055 IMU]
                              [Pendulum on Hinge]
```

A 4-wheeled cart races a track at maximum speed while a reaction motor (brushed DC + gearbox) on the pendulum hinge keeps the pendulum upright. The ESP32 reads the BNO055 IMU (angle + angular velocity) and runs LQR control (with PID as fallback) over a WiFi UDP link from a transmitter PC.

---

## Quick Start

```bash
# 1. Install Python dependencies
pip install -r requirements.txt

# 2. Run the interactive simulation (no hardware needed)
python simulation/pendulum_sim.py

# 3. Open the PID tuner GUI
python tuner/pid_tuner_gui.py

# 4. Open the LQR weight tuner GUI
python tuner/lqr_tuner_gui.py

# 5. Flash the ESP32 (requires PlatformIO)
cd esp32_firmware
pio run --target upload
pio device monitor
```

---

## Modules

### Module 1 — Interactive Simulation (`simulation/pendulum_sim.py`)
Real-time pygame GUI — drive the cart with your keyboard while the control loop stabilises the pendulum.

- **Physics:** `[x, ẋ, θ, θ̇]` integrated at 500 Hz with RK4
- **Phases:** READY → SWING-UP (energy-pumping) → RACE (LQR or PID)
- **Simulated IMU:** mirrors real BNO055 output (euler angle, gyro, linear accel)
- **HUD:** live angle, motor RPM/PWM, cart state, mode

**Keyboard controls**

| Key | Action |
|-----|--------|
| `SPACE` | Start swing-up |
| `← / A` `→ / D` | Drive cart |
| `P` | Toggle LQR ↔ PID |
| `R` | Reset |
| `Q / Esc` | Quit |

---

### Module 2 — LQR Controller (`simulation/lqr_design.py`)
Computes the optimal LQR gain vector **K** from the linearised 2-state pendulum model.

- **State:** `[θ, θ̇]` · **Input:** `τ` (reaction motor torque)
- **Control law:** `τ = −K · [θ, θ̇]`
- Solves the continuous-time ARE via `scipy.linalg.solve_continuous_are`
- Default weights: `Q = diag(120, 20)`, `R = 0.01`

```bash
python simulation/lqr_design.py    # prints A, B, K, eigenvalues
```

Copy the printed K values into `esp32_firmware/lqr_controller.h`, or push them at runtime with the LQR Tuner GUI.

---

### Module 3 — PID Controllers (`simulation/pid_controller.py`)
Cascade angle + angular-rate PID as a fallback.

- **Outer loop:** angle error → angular-velocity setpoint
- **Inner loop:** angular-velocity error → motor torque
- Anti-windup integral clamp, derivative low-pass filter
- Bumpless transfer when switching LQR ↔ PID at runtime

The C++ implementation (`esp32_firmware/pid_controller.cpp`) is a direct port — gains tuned here transfer to hardware without rescaling.

---

### Module 4 — ESP32 Firmware (`esp32_firmware/`)
Full embedded control system.

| File | Purpose |
|------|---------|
| `main.cpp` | Setup, 1 kHz control loop (`esp_timer`), WiFi command handler, safety watchdog |
| `bno055_driver.h/.cpp` | BNO055 I2C wrapper — angle, ω, linear acceleration |
| `motor_control.h/.cpp` | LEDC PWM for reaction motor (MOSFET) + wheel motors |
| `lqr_controller.h/.cpp` | C++ LQR: `τ = −(K₀·θ + K₁·ω)` |
| `pid_controller.h/.cpp` | C++ cascade PID (mirrors Python exactly) |
| `wifi_comm.h/.cpp` | UDP command receiver + JSON telemetry sender |
| `platformio.ini` | PlatformIO build config (Adafruit BNO055 library) |

**First-time setup checklist**
1. Set `WIFI_SSID` and `WIFI_PASSWORD` in `main.cpp`.
2. Run `python simulation/lqr_design.py` and paste K values into `lqr_controller.h` (or push them over WiFi at runtime).
3. Verify pin assignments in `motor_control.h` match your wiring.
4. Flash: `cd esp32_firmware && pio run --target upload`

**Safety watchdog** — reaction motor stops immediately if BNO055 loses connection or `|θ| > 45°`.

---

### Module 5 — WiFi Communication Layer (`transmitter/`)

#### `transmitter/transmitter.py` — PC Keyboard Controller
- Sends `DRIVE`, `MODE`, `LQR`, `PID` commands over UDP (port 4210)
- Receives JSON telemetry and shows it on screen
- Press `L` to push default LQR gains to the ESP32

```bash
python transmitter/transmitter.py [--ip <ESP32_IP>]
```

| Key | Action |
|-----|--------|
| `← / A` `→ / D` | Drive |
| `P` | Toggle LQR ↔ PID |
| `S` | Emergency stop |
| `+` / `-` | Increase/decrease speed |
| `L` | Push default LQR gains |
| `Q / Esc` | Quit |

#### `transmitter/telemetry_dashboard.py` — Live Telemetry Plots
Four scrolling plots: θ, ω, motor PWM, and control-loop timing (µs).

```bash
python transmitter/telemetry_dashboard.py
```

**UDP Protocol**

```
PC → ESP32  (port 4210):
  DRIVE <left_pct> <right_pct>
  MODE LQR | MODE PID
  PID <kpo> <kio> <kdo> <kpi> <kii> <kdi>
  LQR <K0> <K1>
  TORQUE <tau_Nm>          # system ID step test
  STOP

ESP32 → PC  (port 4211, every 20 ms):
  {"t":<ms>,"th":<deg>,"om":<dps>,"pwm":<pct>,"mode":<str>,"dt_us":<us>}
```

---

### Module 6 — System Identification (`sysid/system_identification.py`)
Estimates real physical parameters from hardware data to close the sim-to-real gap.

**Fitted parameters:** `I_hinge`, `m_eff`, `b_pend`

**Procedure:** applies a step torque, records θ(t), fits the linear pendulum ODE via `scipy.optimize.least_squares`, outputs updated LQR gains.

```bash
python sysid/system_identification.py              # offline synthetic test
python sysid/system_identification.py --live --esp32 <IP>
```

---

### Module 7 — Race Mode Controller (`race_mode/race_controller.py`)
Combines maximum cart speed with pendulum stability.

- **Velocity profile:** trapezoidal ramp — accelerate to cruise speed, brake before the end.
- **Feedforward:** predicts pendulum disturbance from planned acceleration:
  `τ_ff = −(m·L/2)·ẍ_cart` — pre-compensates tilt before it builds up.
- **Stability guard:** if `|θ| > 10°`, reduces speed to 30 % of cruise until recovery.

```bash
python race_mode/race_controller.py              # simulation demo + plots
python race_mode/race_controller.py --live --esp32 <IP> [--vmax 2.5]
```

---

### GUI Tuners (`tuner/`)

#### `tuner/pid_tuner_gui.py` — PID Gains Tuner
Move sliders → simulation reruns instantly → plot updates.

- 6 sliders: `Kp/Ki/Kd` for outer loop (angle → ω setpoint) and inner loop (ω error → torque)
- Shows: settling time, peak overshoot, steady-state error
- "Send to ESP32" button pushes gains over UDP

```bash
python tuner/pid_tuner_gui.py [--ip <ESP32_IP>]
```

#### `tuner/lqr_tuner_gui.py` — LQR Weight Tuner
- 3 sliders: `Q_theta` (angle weight), `Q_omega` (velocity weight), `R_torque` (torque penalty)
- Shows computed K gains and closed-loop eigenvalues with stability indicator
- Step-response plot + phase portrait (θ vs θ̇)
- "Send to ESP32" pushes computed K directly

```bash
python tuner/lqr_tuner_gui.py [--ip <ESP32_IP>]
```

---

## Tests

```bash
python -m pytest simulation/tests/ -v         # all tests
python -m pytest simulation/tests/test_controllers.py -v
python -m pytest simulation/tests/test_physics.py -v
```

50 tests covering physics EOM, RK4 integrator, simulated IMU, LQR stability, PID behaviour, and cross-controller comparison.

---

## Recommended Build Order

```
Module 1 (Sim GUI) → Module 2 (LQR) → Module 3 (PID)   ← tune offline
        ↓
    Tuner GUIs (pid_tuner_gui, lqr_tuner_gui)            ← refine gains
        ↓
Module 4 (ESP32 Firmware) + Module 5 (WiFi)              ← bring up hardware
        ↓
Module 6 (System ID)                                      ← fix model mismatch
        ↓
Module 7 (Race Mode)                                      ← competition prep
```

---

## Hardware Checklist

| Component | Detail |
|---|---|
| Microcontroller | ESP32 DevKit |
| IMU | BNO055 (I2C, SDA=21 SCL=22) |
| Wheel motor driver | L298N or DRV8833 |
| Reaction motor | Brushed DC + gearbox, direct ESP32 PWM via MOSFET |
| Communication | WiFi UDP |

---

## Repository Structure

```
MIC_Project/
├── simulation/
│   ├── pendulum_sim.py          # Module 1 — interactive pygame simulation
│   ├── lqr_design.py            # Module 2 — LQR gain computation
│   ├── pid_controller.py        # Module 3 — PID + CascadePID
│   └── tests/                   # pytest test suite (50 tests)
├── esp32_firmware/
│   ├── main.cpp                 # Module 4 — 1 kHz control loop
│   ├── bno055_driver.h/.cpp     # BNO055 IMU driver
│   ├── motor_control.h/.cpp     # PWM motor control
│   ├── lqr_controller.h/.cpp    # LQR controller
│   ├── pid_controller.h/.cpp    # PID + Cascade PID
│   ├── wifi_comm.h/.cpp         # UDP WiFi protocol
│   └── platformio.ini           # PlatformIO build config
├── transmitter/
│   ├── transmitter.py           # Module 5 — PC keyboard + WiFi control
│   └── telemetry_dashboard.py   # Module 5 — live telemetry plots
├── tuner/
│   ├── pid_tuner_gui.py         # PID slider tuner GUI
│   └── lqr_tuner_gui.py         # LQR weight tuner GUI
├── sysid/
│   └── system_identification.py # Module 6 — parameter estimation
├── race_mode/
│   └── race_controller.py       # Module 7 — speed profile + feedforward
├── requirements.txt
├── CLAUDE.md
└── README.md
```
