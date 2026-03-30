#pragma once
/*
 * lqr_controller.h — LQR pendulum stabiliser (ESP32 implementation)
 * ==================================================================
 * Mirrors the Python lqr_design.py control law exactly:
 *
 *   τ = −(K₀ · θ + K₁ · ω)
 *
 * where θ (rad) is the pendulum angle from vertical and ω (rad/s) is the
 * angular velocity.  Both come from the BNO055 driver.
 *
 * Default gains
 * -------------
 * Computed offline by running:
 *   python simulation/lqr_design.py
 * with Q = diag(120, 20), R = 0.01.
 *
 * HOW TO UPDATE GAINS:
 *   1. Run:  python simulation/lqr_design.py
 *   2. Copy the printed K values into the constructor defaults below, OR
 *   3. Send a "LQR <K0> <K1>" UDP command at runtime (no reflash needed).
 *
 * Output is clamped to ±max_torque (default 5 N·m) before being passed to
 * MotorControl::setReactionMotor().
 */

#include <Arduino.h>

class LQRController {
public:
    /*
     * K0          — gain on angle error  (from lqr_design.py output)
     * K1          — gain on angular velocity (from lqr_design.py output)
     * max_torque  — saturation limit (N·m), must match simulation MAX_TORQUE
     *
     * Default K values are approximate; replace with the exact values printed
     * by `python simulation/lqr_design.py` after you run it.
     */
    explicit LQRController(float K0         = 109.5f,
                            float K1         = 14.2f,
                            float max_torque = 5.0f);

    /*
     * Compute reaction motor torque for the current pendulum state.
     *   theta_rad  — angle from vertical (rad), 0 = upright, positive = tilted right
     *   omega_rads — angular velocity (rad/s)
     * Returns torque in N·m, clamped to ±max_torque.
     */
    float compute(float theta_rad, float omega_rads) const;

    /* Update gains at runtime (e.g. from a UDP "LQR <K0> <K1>" command). */
    void setGains(float K0, float K1);

    float getK0()        const { return _K0;         }
    float getK1()        const { return _K1;         }
    float getMaxTorque() const { return _max_torque; }

private:
    float _K0;
    float _K1;
    float _max_torque;

    /* Saturate v to [lo, hi]. */
    static float _clamp(float v, float lo, float hi);
};
