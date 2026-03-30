/*
 * motor_control.cpp — PWM motor driver implementation
 * ====================================================
 * See motor_control.h for pin assignments and usage notes.
 *
 * LEDC channel assignments (0-15 available on ESP32):
 *   CH 0 — reaction motor
 *   CH 1 — left wheel motors
 *   CH 2 — right wheel motors
 * Channels 3-15 are free for other uses.
 *
 * PWM frequency: 20 kHz (inaudible; good for small brushed DC motors).
 * Duty resolution: 8-bit (0 = off, 255 = full power).
 *
 * H-bridge wiring (L298N / DRV8833):
 *   Motor FORWARD:  DIR pin HIGH, PWM pin carries duty cycle.
 *   Motor REVERSE:  DIR pin LOW,  PWM pin carries duty cycle.
 *   Motor BRAKE:    DIR pin HIGH, PWM pin LOW  (or just call stopAll()).
 *
 * Reaction motor wiring (MOSFET):
 *   REACT_DIR_PIN → controls a polarity-switching relay or H-bridge.
 *   If using a simple MOSFET (unidirectional), ignore REACT_DIR_PIN and
 *   set it permanently HIGH; use only positive normalized values.
 */

#include "motor_control.h"

// LEDC channel numbers — must not overlap with any other ledcAttachPin calls
static constexpr int CH_REACT   = 0;
static constexpr int CH_WHEEL_L = 1;
static constexpr int CH_WHEEL_R = 2;

// PWM configuration
static constexpr int PWM_FREQ   = 20000;   // 20 kHz
static constexpr int PWM_RES    = 8;       // 8-bit (0-255)
static constexpr int PWM_MAX    = 255;


void MotorControl::begin() {
    // Initialise LEDC channels for PWM output
    ledcSetup(CH_REACT,   PWM_FREQ, PWM_RES);
    ledcSetup(CH_WHEEL_L, PWM_FREQ, PWM_RES);
    ledcSetup(CH_WHEEL_R, PWM_FREQ, PWM_RES);

    // Attach GPIO pins to LEDC channels
    ledcAttachPin(REACT_PWM_PIN, CH_REACT);
    ledcAttachPin(WHEEL_L_PWM,   CH_WHEEL_L);
    ledcAttachPin(WHEEL_R_PWM,   CH_WHEEL_R);

    // Configure direction pins as digital outputs
    pinMode(REACT_DIR_PIN, OUTPUT);
    pinMode(WHEEL_L_DIR,   OUTPUT);
    pinMode(WHEEL_R_DIR,   OUTPUT);

    // Start with all motors stopped
    stopAll();
}

void MotorControl::_setPwmChannel(float pct, int dir_pin, int ledc_ch) {
    // Clamp to valid range
    if (pct >  100.0f) pct =  100.0f;
    if (pct < -100.0f) pct = -100.0f;

    // Direction: HIGH = forward, LOW = reverse
    bool forward = (pct >= 0.0f);
    digitalWrite(dir_pin, forward ? HIGH : LOW);

    // Duty cycle: map |pct|/100 → 0-255
    int duty = (int)(fabsf(pct) / 100.0f * (float)PWM_MAX);
    if (duty > PWM_MAX) duty = PWM_MAX;

    ledcWrite(ledc_ch, duty);
}

void MotorControl::setReactionMotor(float normalized) {
    // Clamp and convert -1..+1 to percent
    if (normalized >  1.0f) normalized =  1.0f;
    if (normalized < -1.0f) normalized = -1.0f;
    _react_pct = normalized * 100.0f;

    _setPwmChannel(_react_pct, REACT_DIR_PIN, CH_REACT);
}

void MotorControl::setWheelSpeed(float left_pct, float right_pct) {
    _wheel_l_pct = left_pct;
    _wheel_r_pct = right_pct;

    _setPwmChannel(left_pct,  WHEEL_L_DIR, CH_WHEEL_L);
    _setPwmChannel(right_pct, WHEEL_R_DIR, CH_WHEEL_R);
}

void MotorControl::stopAll() {
    // Zero all PWM outputs (brake state)
    ledcWrite(CH_REACT,   0);
    ledcWrite(CH_WHEEL_L, 0);
    ledcWrite(CH_WHEEL_R, 0);

    _react_pct   = 0.0f;
    _wheel_l_pct = 0.0f;
    _wheel_r_pct = 0.0f;
}
