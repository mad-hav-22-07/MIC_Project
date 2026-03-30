/*
 * pid_controller.cpp — PID and Cascade PID implementations
 * =========================================================
 * See pid_controller.h for full documentation.
 *
 * This is a direct C++ port of pid_controller.py.  The numerical behaviour
 * is identical when given the same dt and gain values, so gains tuned in the
 * Python simulation (or the PID tuner GUI) transfer to hardware without scaling.
 */

#include "pid_controller.h"
#include <math.h>

// ─── PIDController ────────────────────────────────────────────────────────────

PIDController::PIDController(float kp, float ki, float kd,
                              float max_output, float deriv_alpha)
    : _kp(kp), _ki(ki), _kd(kd),
      _max_output(max_output),
      // Anti-windup clamp: keep the integral below half the output range
      _max_integral(max_output * 0.5f),
      _deriv_alpha(deriv_alpha),
      _integral(0.0f),
      _prev_error(0.0f),
      _d_filtered(0.0f),
      _initialized(false) {}

float PIDController::compute(float error, float dt) {
    // On the very first call, seed prev_error to avoid a derivative spike
    if (!_initialized) {
        _prev_error  = error;
        _initialized = true;
    }

    // ── Integral term with anti-windup ───────────────────────────────────────
    _integral += error * dt;
    if      (_integral >  _max_integral) _integral =  _max_integral;
    else if (_integral < -_max_integral) _integral = -_max_integral;

    // ── Derivative term with first-order low-pass filter ─────────────────────
    // raw_d = Δerror / dt;  filtered with exponential smoothing.
    // alpha = 0 → no smoothing (pure derivative)
    // alpha = 0.1 → light filter (default; suppresses high-frequency noise)
    float raw_d = (dt > 1e-9f) ? (error - _prev_error) / dt : 0.0f;
    _d_filtered  = _deriv_alpha * _d_filtered + (1.0f - _deriv_alpha) * raw_d;
    _prev_error  = error;

    // ── Combine terms and saturate output ────────────────────────────────────
    float out = _kp * error + _ki * _integral + _kd * _d_filtered;
    if      (out >  _max_output) out =  _max_output;
    else if (out < -_max_output) out = -_max_output;

    return out;
}

void PIDController::reset() {
    _integral    = 0.0f;
    _prev_error  = 0.0f;
    _d_filtered  = 0.0f;
    _initialized = false;
}

void PIDController::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    // Do NOT reset state here — allows bumpless gain changes mid-run.
}


// ─── CascadePIDController ─────────────────────────────────────────────────────

CascadePIDController::CascadePIDController(float kpo, float kio, float kdo,
                                            float kpi, float kii, float kdi,
                                            float max_torque)
    // Outer loop output is an angular-velocity setpoint; give it 2× headroom.
    : _outer(kpo, kio, kdo, max_torque * 2.0f),
      _inner(kpi, kii, kdi, max_torque),
      _max_torque(max_torque) {}

float CascadePIDController::compute(float theta_rad, float omega_rads, float dt) {
    // Outer: angle error → desired angular velocity
    float omega_setpoint = _outer.compute(0.0f - theta_rad, dt);

    // Inner: angular-velocity error → reaction motor torque
    float torque = _inner.compute(omega_setpoint - omega_rads, dt);

    return torque;
}

void CascadePIDController::reset() {
    _outer.reset();
    _inner.reset();
}

void CascadePIDController::setOuterGains(float kp, float ki, float kd) {
    _outer.setGains(kp, ki, kd);
}

void CascadePIDController::setInnerGains(float kp, float ki, float kd) {
    _inner.setGains(kp, ki, kd);
}
