#pragma once
/*
 * motor_control.h — PWM motor driver for wheels + reaction motor
 * ==============================================================
 * Controls two sub-systems:
 *
 *  1. REACTION MOTOR  (pendulum hinge, direct MOSFET drive)
 *     One PWM pin + one direction pin → MOSFET gate → brushed DC + gearbox.
 *     No external motor-driver IC needed (MOSFET handles the current).
 *     API: setReactionMotor(normalized)  where -1.0 … +1.0 maps to full reverse … forward.
 *
 *  2. WHEEL MOTORS  (4-wheel drive, differential left/right)
 *     Two H-bridge channels (left side, right side) using a L298N or DRV8833.
 *     Each channel: one PWM pin + one direction pin.
 *     API: setWheelSpeed(left_pct, right_pct)  where -100 … +100 = full reverse … full forward.
 *
 * PWM parameters
 * --------------
 * Frequency  : 20 kHz  (above audible range, good for small DC motors)
 * Resolution : 8-bit   (0-255 duty cycle values)
 * LEDC channels are assigned internally; do not use channels 0-2 elsewhere.
 *
 * Pin wiring (change the static const values below to match your build)
 * ---------------------------------------------------------------------
 *  REACT_PWM_PIN   — ESP32 GPIO → MOSFET gate (reaction motor speed)
 *  REACT_DIR_PIN   — ESP32 GPIO → direction logic (HIGH/LOW)
 *  WHEEL_L_PWM/DIR — left-side wheel motors (both front & rear share one channel)
 *  WHEEL_R_PWM/DIR — right-side wheel motors
 */

#include <Arduino.h>

class MotorControl {
public:
    // ── Pin assignments ────────────────────────────────────────────────────
    // Change these to match your wiring.
    static constexpr int REACT_PWM_PIN  = 16;
    static constexpr int REACT_DIR_PIN  = 17;
    static constexpr int WHEEL_L_PWM   = 25;
    static constexpr int WHEEL_L_DIR   = 26;
    static constexpr int WHEEL_R_PWM   = 27;
    static constexpr int WHEEL_R_DIR   = 32;

    /*
     * Initialise all PWM channels and direction pins.
     * Call once in setup() before sending any motor commands.
     */
    void begin();

    /*
     * Set reaction motor power.
     *   normalized : -1.0 (full reverse) … 0.0 (stop) … +1.0 (full forward)
     * Values outside ±1 are clamped.
     */
    void setReactionMotor(float normalized);

    /*
     * Set wheel motor speeds independently for left and right sides.
     *   left_pct, right_pct : -100 … +100 (percent of full power)
     * Use equal values for straight driving; opposite signs for turning.
     */
    void setWheelSpeed(float left_pct, float right_pct);

    /* Immediately stop all motors (PWM duty = 0). */
    void stopAll();

    /*
     * Current reaction motor output as a percentage (-100 … +100).
     * Useful for telemetry reporting.
     */
    float getReactionPWM() const { return _react_pct; }

private:
    // Last commanded percentages (for telemetry)
    float _react_pct   = 0.0f;
    float _wheel_l_pct = 0.0f;
    float _wheel_r_pct = 0.0f;

    /*
     * Low-level helper: write a signed percent to a LEDC channel + direction pin.
     *   pct      : -100 … +100
     *   dir_pin  : GPIO for direction
     *   ledc_ch  : LEDC PWM channel (0-15 on ESP32)
     */
    void _setPwmChannel(float pct, int dir_pin, int ledc_ch);
};
