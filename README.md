# Inverted Pendulum Race Car — PWM Control

**A 4-wheeled cart that races a track while keeping an inverted pendulum upright using a reaction wheel driven by PWM.**  
No WiFi. No cloud. Just IMU + PID + PWM.

---

## Repository Structure

```
MIC_Project/
├── mcu/                        # Arduino sketches — upload these to the ESP32
│   ├── imu_reader/             # Step 1: verify IMU is wired correctly, read angles
│   ├── imu_calibration/        # Step 2: calibrate BNO055, save offsets to EEPROM
│   ├── motor_pwm_test/         # Step 3: verify motor wiring and direction
│   ├── pid_balance/            # Step 4: reaction wheel only — tune PID here
│   └── full_balance_car/       # Step 5: full system — base car + reaction wheel PID
├── simulation/                 # Python physics simulation and LQR/PID design
│   ├── pendulum_sim.py         # Interactive pygame simulation
│   ├── lqr_design.py           # LQR gain computation
│   ├── pid_controller.py       # PID + CascadePID classes
│   └── tests/                  # pytest unit tests
├── tuner/                      # Offline GUI sliders for PID and LQR gains
│   ├── pid_tuner_gui.py
│   └── lqr_tuner_gui.py
├── requirements.txt
└── README.md
```

---

## Hardware

| Component | Detail |
|-----------|--------|
| MCU | ESP32 DevKit |
| IMU | BNO055 (I2C — SDA=21, SCL=22) |
| Reaction wheel motor | DRV8833 → GPIO 4, 5 |
| Base car motors (×2) | DRV8833 → GPIO 18, 19, 25, 26 |

---

## Getting Started (Arduino)

### Prerequisites
- Arduino IDE (or PlatformIO)
- [Adafruit BNO055](https://github.com/adafruit/Adafruit_BNO055) library
- [Adafruit Unified Sensor](https://github.com/adafruit/Adafruit_Sensor) library

Install both from **Arduino IDE → Tools → Manage Libraries**.

### Step-by-step

**1. Check IMU wiring**
```
Open: mcu/imu_reader/imu_reader.ino
Upload → open Serial Monitor (115200 baud)
You should see X/Y/Z Euler angles updating at 10 Hz.
```

**2. Calibrate the IMU** *(do this once, offsets save to EEPROM)*
```
Open: mcu/imu_calibration/imu_calibration.ino
Upload → open Serial Monitor (115200 baud)
Slowly rotate the rod in a figure-8 pattern.
Wait until [S:3 G:3 A:3 M:3] appears, then reboot.
```

**3. Test motors**
```
Open: mcu/motor_pwm_test/motor_pwm_test.ino
Upload → Serial Monitor → type 150 (forward) or -150 (reverse)
Verify both wheels spin the correct direction.
```

**4. Tune PID (reaction wheel only)**
```
Open: mcu/pid_balance/pid_balance.ino
Set targetAngle to your measured balance point.
Tuning order: Kp first → Kd to damp oscillation → small Ki last.
Use Arduino Serial Plotter to watch Angle and Output in real time.
```

**5. Run the full system**
```
Open: mcu/full_balance_car/full_balance_car.ino
Adjust Kp/Ki/Kd and targetAngle from Step 4.
Base car drives forward autonomously while the reaction wheel balances the pendulum.
```

---

## Python Simulation

Run this offline to test LQR/PID gains before flashing.

```bash
pip install -r requirements.txt
python simulation/pendulum_sim.py
```

**Keyboard controls:** `SPACE` start, `LEFT/A` `RIGHT/D` drive, `P` toggle LQR↔PID, `R` reset, `Q/ESC` quit.

```bash
# GUI tuners (adjust gains with sliders, see step response live)
python tuner/pid_tuner_gui.py
python tuner/lqr_tuner_gui.py

# Print LQR K gains
python simulation/lqr_design.py
```

---

## PID Tuning Reference

| Parameter | Role | Start value |
|-----------|------|-------------|
| `Kp` | Main restoring force | 300 |
| `Ki` | Removes steady-state offset | 5 |
| `Kd` | Damping — reduces oscillation | 4 |
| `targetAngle` | Physical balance point (degrees) | measure manually |
| `FALL_THRESHOLD` | Abort if error exceeds this | 100° |
| `INTEGRAL_LIMIT` | Anti-windup clamp | 50 |

**PWM dead zone:** motor won't spin below ~70 PWM. `PWM_MIN = 70` maps the PID output so any non-zero effort always reaches this threshold.
