#pragma once
/*
 * bno055_driver.h — BNO055 IMU wrapper for the pendulum hinge
 * ============================================================
 * Wraps the Adafruit BNO055 library to expose only the three values
 * the control loop needs:
 *   • angle_deg   — pendulum tilt from vertical (degrees, 0 = upright)
 *   • omega_degps — angular velocity around the hinge axis (deg/s)
 *   • accel_x     — linear acceleration along the cart track (m/s²)
 *
 * Mounting convention
 * -------------------
 * Mount the BNO055 on the pendulum rod so that:
 *   • Its Y-axis points along the hinge rotation axis (out-of-plane).
 *   • Upright position reads euler.y() ≈ 0 deg (calibrate offset if needed).
 * Adjust the axis selections in bno055_driver.cpp if your mount differs.
 *
 * Usage
 * -----
 *   BNO055Driver imu;
 *   imu.begin();          // call once in setup()
 *   imu.update();         // call every control tick to refresh data
 *   float th = imu.getAngleDeg();
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class BNO055Driver {
public:
    /*
     * address — I2C address of the BNO055.
     *   BNO055_ADDRESS_A (0x28) when ADR pin is LOW  (default)
     *   BNO055_ADDRESS_B (0x29) when ADR pin is HIGH
     *
     * angle_offset_deg — constant added to the raw Euler pitch angle so that
     *   the physical upright position reads 0.  Calibrate once on the bench
     *   by reading imu.getAngleDeg() with the pendulum vertical and negating it.
     */
    explicit BNO055Driver(uint8_t address        = BNO055_ADDRESS_A,
                          float   angle_offset_deg = 0.0f);

    /*
     * Initialise the sensor over I2C (SDA=21, SCL=22 on ESP32 DevKit).
     * Returns true on success.  If false, check wiring and I2C address.
     */
    bool begin();

    /* True if begin() succeeded and the sensor is still responding. */
    bool isConnected() const { return _connected; }

    /*
     * Refresh internal readings from the sensor.
     * Call this once per control loop tick (≥ 1 kHz is fine; BNO055 outputs
     * at 100 Hz internally — repeated reads return the same data until new
     * data is ready, so calling more often than 100 Hz is safe but wasteful).
     */
    void update();

    /* Pendulum tilt angle from vertical (degrees). Positive = tilted right. */
    float getAngleDeg()   const { return _angle_deg;   }

    /* Angular velocity around hinge axis (degrees/s). */
    float getOmegaDegps() const { return _omega_degps; }

    /* Linear acceleration along the cart track direction (m/s²). */
    float getLinAccelX()  const { return _accel_x;     }

private:
    Adafruit_BNO055 _bno;
    bool            _connected;
    float           _angle_offset_deg;

    // Cached readings (updated by update())
    float _angle_deg;
    float _omega_degps;
    float _accel_x;
};
