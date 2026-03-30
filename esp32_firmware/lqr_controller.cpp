/*
 * lqr_controller.cpp — LQR implementation
 * =========================================
 * See lqr_controller.h for the control law and gain update documentation.
 */

#include "lqr_controller.h"
#include <math.h>

LQRController::LQRController(float K0, float K1, float max_torque)
    : _K0(K0), _K1(K1), _max_torque(max_torque) {}

float LQRController::compute(float theta_rad, float omega_rads) const {
    // τ = −(K₀·θ + K₁·ω)
    // Negative sign: positive tilt → negative (restoring) torque.
    float tau = -(_K0 * theta_rad + _K1 * omega_rads);
    return _clamp(tau, -_max_torque, _max_torque);
}

void LQRController::setGains(float K0, float K1) {
    _K0 = K0;
    _K1 = K1;
}

float LQRController::_clamp(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
