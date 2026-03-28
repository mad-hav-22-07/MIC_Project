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

A 4-wheeled cart races a track at maximum speed while a reaction motor (brushed DC + gearbox) on the pendulum hinge keeps the pendulum upright. The ESP32 reads the BNO055 IMU (angle + angular velocity) and runs LQR control (with PID as fallback) over a WiFi link from a transmitter PC. The reaction motor is driven directly from an ESP32 PWM pin via a MOSFET/transistor — no separate motor driver IC needed.

---

## Modules

### Module 1 — Interactive Simulation (Python + pygame)
A real-time GUI simulation of the full system — drive the cart with your keyboard while the control loop stabilizes the pendulum.

- **Renderer:** `pygame` window showing cart on track + pendulum arm, updated in real-time
- **Keyboard control:** Arrow keys / WASD to apply force to the cart (left/right)
- **Physics:** `[x, ẋ, θ, θ̇]` state integrated at each frame using cart-pendulum equations of motion
- **Control loop:** LQR or PID runs live in the sim, computing reaction motor torque to stabilize pendulum
- **Simulated IMU:** Derives `θ`, `θ̇`, and `linear_accel` from physics state to mimic real BNO055 output
- **HUD:** Live readout of angle, angular velocity, motor torque command, active controller mode

**Use when:** Tuning LQR/PID gains offline before flashing the ESP32.

---

### Module 2 — LQR Controller (Python + C++)
Computes optimal gain matrix K from the linearized state-space model.

- **Python:** Derives A, B matrices, solves DARE (discrete algebraic Riccati equation), exports K gains
- **C++:** ESP32 implementation — reads IMU, computes `u = -K*x`, drives reaction motor PWM
- Separate Q/R tuning script to sweep gain parameters

**Primary control strategy.**

---

### Module 3 — PID Controller (Python + C++)
Cascade angle + angular rate PID as fallback.

- Outer loop: angle error → angular velocity setpoint
- Inner loop: angular velocity error → motor PWM
- Python: simulation + Ziegler-Nichols auto-tuner
- C++: ESP32 implementation with bumpless transfer (switch LQR ↔ PID at runtime via WiFi command)

**Use when:** LQR is unstable during initial hardware bring-up, or as competition fallback.

---

### Module 4 — ESP32 Firmware (C++/Arduino)
The full embedded control system.

- BNO055 driver over I2C — angle, angular velocity, linear acceleration
- PWM control for 4 wheel motors (speed + direction)
- Reaction motor: single PWM pin + direction pin → MOSFET → DC motor + gearbox (no driver IC)
- 1 kHz control loop with LQR and PID selectable at runtime
- Watchdog: if IMU fails or `|θ|` exceeds safe threshold → safe-stop reaction motor

**Always needed.**

---

### Module 5 — WiFi Communication Layer
Real-time PC ↔ ESP32 link.

- **ESP32:** UDP server receiving drive commands, sending telemetry
- **Python transmitter:** keyboard/gamepad control for cart speed and direction
- **Telemetry dashboard:** live plots of θ, θ̇, motor PWM, control loop timing

**Always needed.**

---

### Module 6 — System Identification Script (Python)
Estimates real physical parameters from hardware data to close the sim-to-real gap.

- Step-response test on reaction motor → estimates pendulum inertia, motor torque constant
- Logs IMU data, fits model parameters via least squares
- Outputs updated A, B matrices for LQR recomputation

**Use when:** Simulation parameters don't match real hardware behavior (they never do perfectly).

---

### Module 7 — Race Mode Controller (Python + C++)
Combines track speed maximization with pendulum stability.

- Feedforward: predicts pendulum disturbance from planned acceleration profile
- Speed profile planner: max speed within stability margin
- Auto-slowdown if `|θ|` exceeds configurable threshold

**Use when:** Preparing for the actual race.

---

## Recommended Build Order

```
Module 1 (Sim GUI) → Module 2 (LQR) → Module 3 (PID)   ← tune offline
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
| Microcontroller | ESP32 |
| IMU | BNO055 |
| Wheel motor driver | TBD (L298N / DRV8833) |
| Reaction motor | Brushed DC + gearbox, direct ESP32 PWM via MOSFET |
| Communication | WiFi UDP |

---

## Repository Structure (Planned)

```
MIC_Project/
├── simulation/
│   ├── pendulum_sim.py          # Module 1 — interactive pygame simulation
│   ├── lqr_design.py            # Module 2 — LQR gain computation
│   └── pid_tuner.py             # Module 3 — PID simulation + tuner
├── esp32_firmware/
│   ├── main.cpp                 # Module 4 — embedded control loop
│   ├── bno055_driver.cpp/h      # IMU driver
│   ├── motor_control.cpp/h      # PWM wheel + reaction motor (direct GPIO)
│   ├── lqr_controller.cpp/h     # Module 2 — LQR on ESP32
│   └── pid_controller.cpp/h     # Module 3 — PID on ESP32
├── transmitter/
│   ├── transmitter.py           # Module 5 — PC-side WiFi control
│   └── telemetry_dashboard.py   # Module 5 — live telemetry plots
├── sysid/
│   └── system_identification.py # Module 6 — parameter estimation
├── race_mode/
│   └── race_controller.py       # Module 7 — speed + stability planner
└── README.md
```
