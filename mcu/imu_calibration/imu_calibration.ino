#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

/*
 * BNO055 Calibration Sketch
 * --------------------------
 * Run this once before using the PID balance sketch.
 *
 * Calibration procedure:
 *   1. Upload and open Serial Monitor at 115200 baud.
 *   2. Place the robot perfectly still on a flat surface  -> magnetometer calibrates.
 *   3. Slowly rotate the pendulum rod in a figure-8 path  -> gyro/accel calibrate.
 *   4. Wait until [S:3 G:3 A:3 M:3] is shown.
 *   5. The offsets are printed and saved to EEPROM automatically.
 *
 * Calibration status scale: 0 = uncalibrated, 3 = fully calibrated.
 */

#define EEPROM_SIZE       64
#define EEPROM_MAGIC_BYTE 0xAB
#define EEPROM_ADDR       0

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not found. Check wiring and I2C address.");
    while (1);
  }

  bno.setExtCrystalUse(true);
  delay(1000);
  Serial.println("BNO055 found. Starting calibration...");
  Serial.println("Move the rod in a figure-8 until all status values reach 3.");
  Serial.println();
}

void loop() {
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  Serial.print("Calibration  [S:"); Serial.print(sys);
  Serial.print(" G:"); Serial.print(gyro);
  Serial.print(" A:"); Serial.print(accel);
  Serial.print(" M:"); Serial.print(mag);
  Serial.println("]");

  if (sys == 3 && gyro == 3 && accel == 3 && mag == 3) {
    adafruit_bno055_offsets_t offsets;
    bno.getSensorOffsets(offsets);

    Serial.println();
    Serial.println("=== FULLY CALIBRATED ===");
    Serial.println("Offset values (copy these if you want hard-coded calibration):");
    Serial.print("  accel_offset_x: "); Serial.println(offsets.accel_offset_x);
    Serial.print("  accel_offset_y: "); Serial.println(offsets.accel_offset_y);
    Serial.print("  accel_offset_z: "); Serial.println(offsets.accel_offset_z);
    Serial.print("  gyro_offset_x:  "); Serial.println(offsets.gyro_offset_x);
    Serial.print("  gyro_offset_y:  "); Serial.println(offsets.gyro_offset_y);
    Serial.print("  gyro_offset_z:  "); Serial.println(offsets.gyro_offset_z);
    Serial.print("  mag_offset_x:   "); Serial.println(offsets.mag_offset_x);
    Serial.print("  mag_offset_y:   "); Serial.println(offsets.mag_offset_y);
    Serial.print("  mag_offset_z:   "); Serial.println(offsets.mag_offset_z);
    Serial.print("  accel_radius:   "); Serial.println(offsets.accel_radius);
    Serial.print("  mag_radius:     "); Serial.println(offsets.mag_radius);

    // Save magic byte + offsets to EEPROM so the balance sketch can restore them
    EEPROM.write(EEPROM_ADDR, EEPROM_MAGIC_BYTE);
    EEPROM.put(EEPROM_ADDR + 1, offsets);
    EEPROM.commit();
    Serial.println();
    Serial.println("Offsets saved to EEPROM. You can now upload pid_balance.ino.");

    while (1); // stop here — reboot to re-run
  }

  delay(500);
}
