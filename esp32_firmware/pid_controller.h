#pragma once
/*
 * pid_controller.h — PID controllers (ESP32 implementation)
 * ==========================================================
 * Mirrors pid_controller.py exactly so gains tuned in the Python simulation
 * transfer directly to hardware without rescaling.
 *
 * Two classes:
 *
 *  PIDController
 *  -------------
 *  Single-loop discrete PID with:
 *    • Anti-windup integral clamp (max_integral = max_output/2 by default)
 *    • First-order derivative low-pass filter  (alpha=0 → pure D, 1 → no D)
 *    • Symmetric output saturation to max_output
 *
 *  CascadePIDController
 *  --------------------
 *  Two-loop cascade for pendulum stabilisation — same structure as Python:
 *    Outer: angle error (rad) → angular-velocity setpoint (rad/s)
 *    Inner: angular-velocity error → reaction motor torque (N·m)
 *
 *  Default gains match the Python CascadePID defaults.  Override with
 *  setOuterGains() / setInnerGains() or via the "PID ..." UDP command.
 */

#include <Arduino.h>

// ─── Single PID loop ─────────────────────────────────────────────────────────

class PIDController {
public:
    /*
     * kp, ki, kd       — proportional, integral, derivative gains
     * max_output        — symmetric saturation limit (set to 1e9f for none)
     * deriv_alpha       — derivative smoothing factor (0 = raw, 0.1 = light filter)
     */
    explicit PIDController(float kp          = 0.0f,
                            float ki          = 0.0f,
                            float kd          = 0.0f,
                            float max_output  = 1e9f,
                            float deriv_alpha = 0.1f);

    /*
     * Advance the PID by one timestep.
     *   error — setpoint minus measured value
     *   dt    — elapsed time since last call (seconds)
     * Returns the saturated PID output.
     */
    float compute(float error, float dt);

    /* Clear integrator and derivative memory (call after mode switches). */
    void reset();

    /* Update gains at runtime without resetting integrator state. */
    void setGains(float kp, float ki, float kd);

private:
    float _kp, _ki, _kd;
    float _max_output;
    float _max_integral;    // = max_output * 0.5 (set in constructor)
    float _deriv_alpha;

    float _integral;
    float _prev_error;
    float _d_filtered;
    bool  _initialized;     // false until first compute() call (no derivative spike)
};


// ─── Cascade PID (angle → ω setpoint → torque) ───────────────────────────────

class CascadePIDController {
public:
    /*
     * Default gains match the Python CascadePID defaults.
     * max_torque — reaction motor saturation (N·m), must match MAX_TORQUE in sim.
     */
    explicit CascadePIDController(float kp_outer  = 20.0f,
                                   float ki_outer  =  0.0f,
                                   float kd_outer  =  0.0f,
                                   float kp_inner  =  1.5f,
                                   float ki_inner  =  0.0f,
                                   float kd_inner  =  0.0f,
                                   float max_torque =  5.0f);

    /*
     * Compute reaction motor torque for the current pendulum state.
     *   theta_rad  — pendulum angle (rad), 0 = upright
     *   omega_rads — pendulum angular velocity (rad/s)
     *   dt         — timestep (s), normally 0.001 for 1 kHz loop
     */
    float compute(float theta_rad, float omega_rads, float dt);

    /* Reset both inner and outer integrators (call on mode switches). */
    void reset();

    /* Update gains at runtime (e.g. from a "PID ..." UDP command). */
    void setOuterGains(float kp, float ki, float kd);
    void setInnerGains(float kp, float ki, float kd);

private:
    PIDController _outer;   // angle error → angular-velocity setpoint
    PIDController _inner;   // angular-velocity error → torque
    float         _max_torque;
};
