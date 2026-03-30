/*
 * bno055_driver.cpp — BNO055 IMU driver implementation
 * =====================================================
 * See bno055_driver.h for full documentation.
 *
 * Axis mapping note
 * -----------------
 * The BNO055's Euler output depends heavily on how the chip is physically
 * mounted on the pendulum rod.  The defaults here assume:
 *   • BNO055 flat side faces away from the hinge axis.
 *   • Y-axis of the chip aligns with the hinge rotation axis.
 *   • euler.y() increases as the pendulum tilts forward (away from vertical).
 *
 * If your mounting differs, change which euler component is read for angle and
 * which gyro component is read for omega.  Common alternatives:
 *   • Hinge on X-axis: use euler.z() for angle, gyro.x() for omega
 *   • Hinge on Z-axis: use euler.x() for angle, gyro.z() for omega
 *
 * Calibration
 * -----------
 * BNO055 has an internal calibration state.  For stable angle readings:
 *   1. Power on with the pendulum held still (magnetometer samples ambient field).
 *   2. Slowly rotate the pendulum rod through several orientations to calibrate gyro.
 *   3. Wait until Serial shows calibration status [G:3 A:3 M:3 S:3].
 * The sensor retains calibration in EEPROM after power-off if setExtCrystalUse(true).
 */

#include "bno055_driver.h"

BNO055Driver::BNO055Driver(uint8_t address, float angle_offset_deg)
    : _bno(55, address),
      _connected(false),
      _angle_offset_deg(angle_offset_deg),
      _angle_deg(0.0f),
      _omega_degps(0.0f),
      _accel_x(0.0f) {}

bool BNO055Driver::begin() {
    // SDA = GPIO 21, SCL = GPIO 22 (standard ESP32 DevKit pins)
    Wire.begin(21, 22);

    _connected = _bno.begin();
    if (!_connected) {
        return false;
    }

    // Use the external 32.768 kHz crystal on the BNO055 breakout board for
    // improved accuracy and to retain calibration between power cycles.
    delay(1000);   // allow sensor to settle after power-on
    _bno.setExtCrystalUse(true);

    return true;
}

void BNO055Driver::update() {
    if (!_connected) return;

    // ── Euler angles (degrees) ───────────────────────────────────────────────
    // VECTOR_EULER → [heading/yaw, pitch, roll] from the sensor's fusion engine.
    // We use pitch (index 1 = .y()) as the pendulum tilt angle.
    // See the axis mapping note in the file header if you need a different axis.
    imu::Vector<3> euler = _bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    _angle_deg = euler.y() + _angle_offset_deg;

    // ── Gyroscope (deg/s) ────────────────────────────────────────────────────
    // VECTOR_GYROSCOPE gives angular velocity in deg/s in sensor frame.
    // Y-axis corresponds to the hinge rotation (pitch rate).
    imu::Vector<3> gyro = _bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    _omega_degps = gyro.y();

    // ── Linear acceleration (m/s²) ───────────────────────────────────────────
    // VECTOR_LINEARACCEL subtracts gravity from the raw accelerometer, giving
    // only the acceleration due to cart motion.  X-axis is along the track.
    imu::Vector<3> accel = _bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    _accel_x = accel.x();
}
