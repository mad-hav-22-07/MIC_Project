#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// --- Reaction Wheel Pins ---
const int AIN1 = 4;
const int AIN2 = 5;

const int PWM_MIN = 70;
const int PWM_MAX = 255;

// --- PID Tuning ---
// Tuning order: raise Kp until oscillation, back off 20%, add Kd to damp, tiny Ki last.
float Kp = 2000.0;
float Ki =  200.0;
float Kd =    2.0;

const float INTEGRAL_LIMIT  = 50.0;
const float FALL_THRESHOLD  = 100.0; // degrees — stop if tilted too far

float targetAngle   = 85.37; // balance point in degrees (tune this first)
float integral      = 0.0;
float previousError = 0.0;
unsigned long lastTime = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  if (!bno.begin()) {
    Serial.println("BNO055 not found! Check wiring.");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);

  Serial.println("IMU ready. Starting PID...");
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;
  if (dt <= 0) dt = 0.01f;

  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  float currentAngle = orientationData.orientation.y;
  float side         = orientationData.orientation.z;

  float error = targetAngle - currentAngle;
  if (side > 0) error *= -1;

  // Fall detection — stop and reset
  if (abs(error) > FALL_THRESHOLD) {
    driveMotor(0);
    integral = 0;
    previousError = 0;
    Serial.print("FELL | angle="); Serial.print(currentAngle);
    Serial.print(", error="); Serial.println(error);
    delay(300);
    return;
  }

  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float derivative = (error - previousError) / dt;

  // Anti-windup: reset integrator if it saturates
  if (abs(integral) > INTEGRAL_LIMIT - 0.1) {
    integral   = 0;
    derivative = 0;
    error      = 0;
  }

  previousError = error;
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  driveMotor(output);

  // Serial Plotter output
  Serial.print("Target:");  Serial.print(targetAngle);
  Serial.print(",Angle:");  Serial.print(currentAngle);
  Serial.print(",Error:");  Serial.print(error);
  Serial.print(",Output:"); Serial.println(output);

  delay(10); // 100 Hz
}

void driveMotor(float effort) {
  effort = constrain(effort, -50000.0f, 50000.0f);

  if (abs(effort) < 10) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    return;
  }

  int pwmVal = map(abs(effort), 10, 50000, PWM_MIN, PWM_MAX);
  pwmVal = constrain(pwmVal, PWM_MIN, PWM_MAX);

  if (effort > 0) {
    analogWrite(AIN1, pwmVal);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    analogWrite(AIN2, pwmVal);
  }
}
