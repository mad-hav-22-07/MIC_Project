#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {
  Serial.begin(115200);

  if (!bno.begin()) {
    Serial.println("BNO055 not found! Check wiring.");
    while (1);
  }

  bno.setExtCrystalUse(true);
  delay(1000);

  Serial.println("IMU Ready.");
  Serial.println("Stand the robot up and find the balance point!");
}

void loop() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  float x = orientationData.orientation.x;
  float y = orientationData.orientation.y;
  float z = orientationData.orientation.z;

  // Serial Plotter friendly labels
  Serial.print("Tilt_Angle:"); Serial.print(z);
  Serial.print(" X:"); Serial.print(x);
  Serial.print(" Y:"); Serial.print(y);
  Serial.print(" Z:"); Serial.println(z);

  delay(100); // 10 Hz
}
